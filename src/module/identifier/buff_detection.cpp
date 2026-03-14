
#include "module/identifier/buff_detection.hpp"
#include "rmcs_msgs/robot_color.hpp"
#include "utility/image/image.details.hpp"
#include "utility/image/image.hpp"

#include "utility/shared/context.hpp"
#include "vc/core/type_expansion.hpp"
#include "vc/dataio/dataio.h"
#include "vc/detector/detector_input.h"
#include "vc/detector/detector_output.h"
#include "vc/detector/rune_detector.h"
#include "vc/feature/feature_node.h"
#include "vc/feature/rune_combo.h"
#include "vc/feature/rune_group.h"
#include "vc/feature/rune_tracker.h"
#include "vc/feature/tracking_feature_node.h"

#include <memory>
#include <opencv2/core/utility.hpp>
#include <vector>

using namespace std;
using namespace cv;

namespace rmcs::identifier {

struct BuffDetection::BuffDetectionFrame {
    const unique_ptr<rmcs::Image>& image;
    const GyroData& gyro_data;
    const util::ControlState& control_state;
};

struct BuffDetection::BuffDetectionResult {
    shared_ptr<RuneTracker> target_tracker;
};

struct BuffDetection::Impl {

    auto initialize([[maybe_unused]] const YAML::Node& yaml) noexcept
        -> std::expected<void, std::string> {
        return { };
    }

    shared_ptr<RuneTracker> buff_detect(const BuffDetectionFrame& frame) {
        static auto rune_groups   = vector<FeatureNode_ptr> { };
        static auto rune_detector = RuneDetector::make_detector();

        DetectorInput input;
        DetectorOutput output;

        input.setImage(frame.image->details().mat);
        input.setGyroData(frame.gyro_data);
        input.setTick(frame.control_state.timestamp.time_since_epoch().count());
        input.setColor(frame.control_state.color == rmcs_msgs::RobotColor::BLUE ? PixChannel::BLUE
                                                                                : PixChannel::RED);
        input.setFeatureNodes(rune_groups);

        rune_detector->detect(input, output);

        if (!output.getValid()) {
            return nullptr;
        }

        rune_groups = output.getFeatureNodes();
        if (rune_groups.empty()) {
            return nullptr;
        }

        auto rune_group = RuneGroup::cast(rune_groups.front());
        if (rune_group == nullptr || rune_group->childFeatures().empty()) {
            return nullptr;
        }

        FeatureNode_ptr target_tracker = nullptr;
        for (const auto& tracker : rune_group->getTrackers()) {
            auto _tracker = TrackingFeatureNode::cast(tracker);
            if (_tracker == nullptr) {
                continue;
            }
            if (_tracker->getHistoryNodes().empty()) {
                continue;
            }
            auto rune_combo = RuneCombo::cast(_tracker->getHistoryNodes().front());
            if (rune_combo == nullptr) {
                continue;
            }
            auto type = rune_combo->getRuneType();
            if (type == RuneType::PENDING_STRUCK) {
                target_tracker = _tracker;
                break;
            }
        }
        if (target_tracker == nullptr) {
            return nullptr;
        }
        return RuneTracker::cast(target_tracker);
    }
};

BuffDetection::BuffDetection() noexcept
    : pimpl { std::make_unique<Impl>() } { }

BuffDetection::~BuffDetection() noexcept = default;

auto BuffDetection::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto BuffDetection::auto_detect(const BuffDetectionFrame& frame) noexcept
    -> std::optional<BuffDetectionResult> {
    auto result = pimpl->buff_detect(frame);
    if (!result) {
        return std::nullopt;
    }
    return BuffDetectionResult { std::move(result) };
}

}