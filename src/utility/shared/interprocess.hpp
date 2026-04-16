#pragma once
#include <array>
#include <atomic>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <sys/mman.h>
#include <type_traits>
#include <unistd.h>

namespace rmcs::shm {

namespace detail {

    template <typename Context, std::size_t Len>
    class MappedContext final {
    public:
        MappedContext()                                        = default;
        MappedContext(const MappedContext&)                    = delete;
        auto operator=(const MappedContext&) -> MappedContext& = delete;

        ~MappedContext() noexcept {
            if (context_ != nullptr) {
                munmap(static_cast<void*>(context_), Len);
            }
            if (shm_fd_ != -1) {
                close(shm_fd_);
            }
        }

        auto open_sender(const char* id) noexcept -> bool {
            return open(id, O_CREAT | O_RDWR, true);
        }

        auto open_receiver(const char* id) noexcept -> bool { return open(id, O_RDWR, false); }

        auto opened() const noexcept -> bool { return context_ != nullptr; }

        auto get() const noexcept -> Context* { return context_; }

    private:
        auto open(const char* id, int flags, bool resize) noexcept -> bool {
            if (opened()) return true;

            auto fd = shm_open(id, flags, 0666);
            if (fd == -1) {
                return false;
            }

            if (resize && ftruncate(fd, Len) == -1) {
                close(fd);
                return false;
            }

            auto* shm_ptr = mmap(nullptr, Len, PROT_WRITE | PROT_READ, MAP_SHARED, fd, 0);
            if (shm_ptr == MAP_FAILED) {
                close(fd);
                return false;
            }

            shm_fd_  = fd;
            context_ = static_cast<Context*>(shm_ptr);
            return true;
        }

        int shm_fd_ { -1 };
        Context* context_ { nullptr };
    };

} // namespace detail

template <typename T>
struct alignas(64) SharedContext final {

    alignas(64) std::atomic<std::uint64_t> version;
    alignas(64) T data;
    std::byte padding[64 - sizeof(T) % 64];

    static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");
};

template <typename T>
struct Client {
    using Context = SharedContext<T>;

    static constexpr auto kContextLen = sizeof(Context);

    class Send final {
    public:
        Send()                               = default;
        Send(const Send&)                    = delete;
        auto operator=(const Send&) -> Send& = delete;

        auto open(const char* id) noexcept -> bool { return context_.open_sender(id); }
        auto opened() const noexcept { return context_.opened(); }
        auto send(const T& data) noexcept -> void {
            with_write([&](T& shared) { shared = data; });
        }

        template <typename F>
        auto with_write(F&& fn) noexcept -> void
            requires std::invocable<F, T&>
        {
            auto* context = context_.get();
            if (!context) return;

            context->version.fetch_add(1, std::memory_order::acq_rel);
            fn(context->data);
            context->version.fetch_add(1, std::memory_order::acq_rel);
        }

    private:
        detail::MappedContext<Context, kContextLen> context_ {};
    };

    class Recv {
    public:
        Recv()                               = default;
        Recv(const Recv&)                    = delete;
        auto operator=(const Recv&) -> Recv& = delete;

        auto open(const char* id) noexcept -> bool { return context_.open_receiver(id); }
        auto opened() const noexcept { return context_.opened(); }
        auto recv(T& out_data) const noexcept -> void {
            with_read([&](const T& shared) { out_data = shared; });
        }

        template <typename F>
        auto with_read(F&& fn) const noexcept -> void
            requires std::invocable<F, const T&>
        {
            auto* context = context_.get();
            if (!context) return;

            auto snapshot = T {};
            auto version1 = std::uint64_t {};
            auto version2 = std::uint64_t {};

            do {
                version1 = context->version.load(std::memory_order::acquire);
                snapshot = context->data;
                version2 = context->version.load(std::memory_order::acquire);
            } while ((version1 != version2) || (version1 & 1));

            version = version2;
            fn(snapshot);
        }

        auto is_updated() const noexcept -> bool {
            auto* context = context_.get();
            if (!context) return false;

            auto current = context->version.load(std::memory_order::acquire);
            return (current != version) && !(current & 1);
        }

    private:
        mutable std::uint64_t version { 0 };

        detail::MappedContext<Context, kContextLen> context_ {};
    };
};

template <typename T, std::size_t N>
struct HistoryClient {
    static_assert(N > 0, "History capacity must be non-zero");
    static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");

    struct alignas(64) Entry final {
        alignas(64) std::atomic<std::uint64_t> version;
        std::uint64_t sequence;
        T data;
    };

    struct alignas(64) Context final {
        alignas(64) std::atomic<std::uint64_t> committed;
        alignas(64) std::array<Entry, N> entries;
    };

    static constexpr auto kContextLen = sizeof(Context);

    class Send final {
    public:
        Send()                               = default;
        Send(const Send&)                    = delete;
        auto operator=(const Send&) -> Send& = delete;

        auto open(const char* id) noexcept -> bool {
            if (!context_.open_sender(id)) {
                return false;
            }

            auto* context = context_.get();
            next_sequence = context->committed.load(std::memory_order::acquire);
            return true;
        }

        auto opened() const noexcept { return context_.opened(); }

        auto push(const T& data) noexcept -> bool {
            auto* context = context_.get();
            if (!context) return false;

            const auto sequence = next_sequence++;
            auto& entry         = context->entries[sequence % N];

            entry.version.fetch_add(1, std::memory_order::acq_rel);
            entry.sequence = sequence;
            entry.data     = data;
            entry.version.fetch_add(1, std::memory_order::acq_rel);

            context->committed.store(next_sequence, std::memory_order::release);
            return true;
        }

    private:
        detail::MappedContext<Context, kContextLen> context_ {};
        std::uint64_t next_sequence { 0 };
    };

    class Recv final {
    public:
        Recv()                               = default;
        Recv(const Recv&)                    = delete;
        auto operator=(const Recv&) -> Recv& = delete;

        auto open(const char* id) noexcept -> bool {
            if (!context_.open_receiver(id)) {
                return false;
            }

            auto* context         = context_.get();
            const auto committed  = context->committed.load(std::memory_order::acquire);
            observed_committed    = 0;
            next_sequence_to_pop_ = committed;
            return true;
        }

        auto opened() const noexcept { return context_.opened(); }

        auto is_updated() const noexcept -> bool {
            auto* context = context_.get();
            if (!context) return false;
            return context->committed.load(std::memory_order::acquire) != observed_committed;
        }

        auto latest(T& out_data) const noexcept -> bool {
            return find_latest([](const T&) { return true; }, out_data);
        }

        template <typename Predicate>
        auto find_latest(Predicate&& predicate, T& out_data) const noexcept -> bool {
            auto* context = context_.get();
            if (!context) return false;

            const auto committed = context->committed.load(std::memory_order::acquire);
            const auto oldest    = oldest_sequence(committed);

            for (auto sequence = committed; sequence > oldest; --sequence) {
                auto candidate = T {};
                if (!read_sequence(sequence - 1, candidate)) {
                    continue;
                }
                if (predicate(candidate)) {
                    out_data           = candidate;
                    observed_committed = committed;
                    return true;
                }
            }

            observed_committed = committed;
            return false;
        }

        auto pop_next(T& out_data) const noexcept -> bool {
            auto* context = context_.get();
            if (!context) return false;

            const auto committed = context->committed.load(std::memory_order::acquire);
            const auto oldest    = oldest_sequence(committed);

            if (next_sequence_to_pop_ < oldest) {
                next_sequence_to_pop_ = oldest;
                observed_committed    = committed;
                return false;
            }

            if (next_sequence_to_pop_ >= committed) {
                observed_committed = committed;
                return false;
            }

            if (!read_sequence(next_sequence_to_pop_, out_data)) {
                return false;
            }

            ++next_sequence_to_pop_;
            observed_committed = committed;
            return true;
        }

    private:
        static auto oldest_sequence(std::uint64_t committed) noexcept -> std::uint64_t {
            return committed > N ? committed - N : 0;
        }

        auto read_sequence(std::uint64_t sequence, T& out_data) const noexcept -> bool {
            auto* context = context_.get();
            if (!context) return false;

            const auto& entry = context->entries[sequence % N];

            auto version1        = std::uint64_t {};
            auto version2        = std::uint64_t {};
            auto stored_sequence = std::uint64_t {};
            auto candidate       = T {};

            do {
                version1        = entry.version.load(std::memory_order::acquire);
                stored_sequence = entry.sequence;
                candidate       = entry.data;
                version2        = entry.version.load(std::memory_order::acquire);
            } while ((version1 != version2) || (version1 & 1));

            if (stored_sequence != sequence) {
                return false;
            }

            out_data = candidate;
            return true;
        }

        mutable std::uint64_t observed_committed { 0 };
        mutable std::uint64_t next_sequence_to_pop_ { 0 };

        detail::MappedContext<Context, kContextLen> context_ {};
    };
};

} // namespace rmcs::shm
