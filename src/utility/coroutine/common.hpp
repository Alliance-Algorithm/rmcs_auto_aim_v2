#pragma once
#include <coroutine>
#include <optional>

namespace rmcs::co {

struct task_extended {
    auto done(this auto& self) { return self.coroutine.done(); }
    auto resume(this auto& self) {
        if (self.coroutine && !self.done()) self.coroutine.resume();
    }
};

template <typename T>
struct [[nodiscard]] task : public task_extended {

    struct promise_type {
        using coroutine = std::coroutine_handle<promise_type>;
        std::optional<T> result;

        auto get_return_object() noexcept
            requires std::constructible_from<task, coroutine>
        {
            return task { coroutine::from_promise(*this) };
        }

        auto return_value(T value) noexcept {
            this->result = std::move(value); //
        }

        static auto initial_suspend() noexcept { return std::suspend_never {}; }
        static auto final_suspend() noexcept { return std::suspend_never {}; }

        static auto unhandled_exception() { }
    };
    using handler = std::coroutine_handle<promise_type>;

    auto result(this auto& self) noexcept -> decltype(auto) {
        promise_type& promise = self.coroutine.promise();
        return *promise.result;
    }

    explicit task(std::coroutine_handle<promise_type> h) noexcept
        : coroutine { h } { }

    ~task() noexcept {
        if (coroutine) coroutine.destroy();
    }

    std::coroutine_handle<promise_type> coroutine;
};

template <>
struct [[nodiscard]] task<void> : task_extended {

    struct promise_type {
        using coroutine = std::coroutine_handle<promise_type>;

        auto get_return_object() noexcept { return task<void> { coroutine::from_promise(*this) }; }

        static auto return_void() noexcept { }
        static auto unhandled_exception() { }

        static auto initial_suspend() noexcept { return std::suspend_never {}; }
        static auto final_suspend() noexcept { return std::suspend_never {}; }
    };
    using handler = std::coroutine_handle<promise_type>;

    explicit task(std::coroutine_handle<promise_type> co) noexcept
        : coroutine { co } { }

    ~task() noexcept {
        if (coroutine) coroutine.destroy();
    }

    std::coroutine_handle<promise_type> coroutine;
};

}
