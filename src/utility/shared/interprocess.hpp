#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <concepts>
#include <cstdint>
#include <fcntl.h>
#include <sys/mman.h>
#include <type_traits>
#include <unistd.h>

namespace rmcs::shm {

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
    static constexpr auto kDataLen    = sizeof(T);

    class Send final {
    public:
        ~Send() noexcept {
            if (context) {
                munmap(static_cast<void*>(context), kContextLen);
            }
            if (shm_fd != -1) {
                close(shm_fd);
            }
        }
        auto open(const char* id) noexcept -> bool {
            shm_fd = shm_open(id, O_CREAT | O_RDWR, 0666);
            if (shm_fd == -1) {
                return false;
            }
            if (ftruncate(shm_fd, kContextLen) == -1) {
                close(shm_fd);
                return false;
            }

            auto* shm_ptr =
                mmap(nullptr, kContextLen, PROT_WRITE | PROT_READ, MAP_SHARED, shm_fd, 0);
            if (shm_ptr == MAP_FAILED) {
                close(shm_fd);
                return false;
            }

            context = static_cast<Context*>(shm_ptr);
            return true;
        }
        auto opened() const noexcept { return context != nullptr; }
        auto send(const T& data) noexcept -> void {
            if (!context) return;
            context->version.fetch_add(1, std::memory_order::acq_rel);
            context->data = data;
            context->version.fetch_add(1, std::memory_order::acq_rel);
        }

        template <typename F>
        auto with_write(F&& fn) noexcept -> void
            requires std::invocable<F, T&>
        {
            if (!context) return;

            context->version.fetch_add(1, std::memory_order::acq_rel);
            fn(context->data);
            context->version.fetch_add(1, std::memory_order::acq_rel);
        }

    private:
        int shm_fd { -1 };
        Context* context { nullptr };
    };

    class Recv {
    public:
        ~Recv() noexcept {
            if (context) {
                munmap(static_cast<void*>(context), kContextLen);
            }
            if (shm_fd != -1) {
                close(shm_fd);
            }
        }

        auto open(const char* id) noexcept -> bool {
            shm_fd = shm_open(id, O_RDWR, 0666);
            if (shm_fd == -1) {
                return false;
            }

            auto* shm_ptr =
                mmap(nullptr, kContextLen, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
            if (shm_ptr == MAP_FAILED) {
                close(shm_fd);
                return false;
            }

            context = static_cast<Context*>(shm_ptr);
            return true;
        }
        auto opened() const noexcept { return context != nullptr; }
        auto recv(T& out_data) const noexcept -> void {
            if (!context) return;

            auto version1 = std::uint64_t {};
            auto version2 = std::uint64_t {};

            do {
                version1 = context->version.load(std::memory_order::acquire);
                out_data = context->data;
                version2 = context->version.load(std::memory_order::acquire);
            } while ((version1 != version2) || (version1 & 1));

            version = version2;
        }

        template <typename F>
        auto with_read(F&& fn) const noexcept -> void
            requires std::invocable<F, const T&>
        {
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
            if (!context) return false;

            auto current = context->version.load(std::memory_order::acquire);
            return (current != version) && !(current & 1);
        }

    private:
        mutable std::uint64_t version { 0 };

        int shm_fd { -1 };
        Context* context { nullptr };
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
        ~Send() noexcept {
            if (context) {
                munmap(static_cast<void*>(context), kContextLen);
            }
            if (shm_fd != -1) {
                close(shm_fd);
            }
        }

        auto open(const char* id) noexcept -> bool {
            shm_fd = shm_open(id, O_CREAT | O_RDWR, 0666);
            if (shm_fd == -1) {
                return false;
            }
            if (ftruncate(shm_fd, kContextLen) == -1) {
                close(shm_fd);
                return false;
            }

            auto* shm_ptr =
                mmap(nullptr, kContextLen, PROT_WRITE | PROT_READ, MAP_SHARED, shm_fd, 0);
            if (shm_ptr == MAP_FAILED) {
                close(shm_fd);
                return false;
            }

            context       = static_cast<Context*>(shm_ptr);
            next_sequence = context->committed.load(std::memory_order::acquire);
            return true;
        }

        auto opened() const noexcept { return context != nullptr; }

        auto push(const T& data) noexcept -> bool {
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
        int shm_fd { -1 };
        Context* context { nullptr };
        std::uint64_t next_sequence { 0 };
    };

    class Recv final {
    public:
        ~Recv() noexcept {
            if (context) {
                munmap(static_cast<void*>(context), kContextLen);
            }
            if (shm_fd != -1) {
                close(shm_fd);
            }
        }

        auto open(const char* id) noexcept -> bool {
            shm_fd = shm_open(id, O_RDWR, 0666);
            if (shm_fd == -1) {
                return false;
            }

            auto* shm_ptr =
                mmap(nullptr, kContextLen, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
            if (shm_ptr == MAP_FAILED) {
                close(shm_fd);
                return false;
            }

            context               = static_cast<Context*>(shm_ptr);
            const auto committed  = context->committed.load(std::memory_order::acquire);
            observed_committed    = 0;
            next_sequence_to_pop_ = committed;
            return true;
        }

        auto opened() const noexcept { return context != nullptr; }

        auto is_updated() const noexcept -> bool {
            if (!context) return false;
            return context->committed.load(std::memory_order::acquire) != observed_committed;
        }

        auto latest(T& out_data) const noexcept -> bool {
            return find_latest([](const T&) { return true; }, out_data);
        }

        template <typename Predicate>
        auto find_latest(Predicate&& predicate, T& out_data) const noexcept -> bool {
            if (!context) return false;

            const auto committed = context->committed.load(std::memory_order::acquire);
            const auto oldest    = oldest_sequence(committed);

            for (auto sequence = committed; sequence > oldest; --sequence) {
                auto candidate = T {};
                if (!read_sequence(sequence - 1, candidate)) {
                    continue;
                }
                if (predicate(candidate)) {
                    out_data            = candidate;
                    observed_committed  = committed;
                    return true;
                }
            }

            observed_committed = committed;
            return false;
        }

        auto pop_next(T& out_data) const noexcept -> bool {
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
            if (!context) return false;

            const auto& entry = context->entries[sequence % N];

            auto version1         = std::uint64_t {};
            auto version2         = std::uint64_t {};
            auto stored_sequence  = std::uint64_t {};
            auto candidate        = T {};

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

        int shm_fd { -1 };
        Context* context { nullptr };
    };
};

} // namespace rmcs::shm
