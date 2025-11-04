#pragma once
#include "utility/image.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/color.hpp"
#include "utility/robot/id.hpp"

#include <opencv2/core/mat.hpp>

#include <coroutine>
#include <expected>
#include <functional>

namespace rmcs::identifier {

struct NeuralNetwork final {
    RMCS_PIMPL_DEFINITION(NeuralNetwork)

public:
    using handle_type = std::coroutine_handle<>;

    using ArmorId   = DeviceId;
    using ArmorRect = cv::Rect;

    struct Config {
        CampColor color = CampColor::UNKNOWN;

        cv::Size2i input_size = { 1440, 1080 };
        cv::Size2i infer_size = { 640, 640 };

        double confidence_threshold          = 0.65;
        double non_max_suppression_threshold = 0.45;
    };

    struct Armor {
        ArmorId id = ArmorId::UNKNOWN;
        ArmorRect rect;
    };
    using Result = std::expected<std::vector<Armor>, std::string>;

    auto configure(const Config&) noexcept -> std::expected<void, std::string>;

    auto load_from_filesystem(const std::string& location) noexcept
        -> std::expected<void, std::string_view>;

    auto sync_infer(const Image&) const noexcept -> std::vector<Armor>;

    using Callback = std::function<void(std::unique_ptr<Result>)>;
    auto async_infer(std::shared_ptr<Image const>, Callback) noexcept -> void;

    struct AsyncResult final {
        NeuralNetwork& network;
        std::shared_ptr<Image const> readonly;

        std::unique_ptr<Result> result = nullptr;

        using ResultUnique = std::unique_ptr<Result>;
        auto await_resume() noexcept -> ResultUnique { return std::move(result); }

        auto await_suspend(handle_type handle) noexcept -> void {
            auto callback = [=, this](ResultUnique _result) {
                result = std::move(_result), handle.resume();
            };
            network.async_infer(std::move(readonly), std::move(callback));
        }

        static constexpr auto await_ready() noexcept { return false; }
    };
    auto await_infer(std::shared_ptr<Image const> readonly) noexcept -> AsyncResult {
        return AsyncResult {
            .network  = *this,
            .readonly = std::move(readonly),
        };
    }
};

}
