
#include "module/identifier/buff_detection.hpp"
#include "utility/image/image.details.hpp"
#include "utility/image/image.hpp"

#include "utility/shared/context.hpp"
#include "vc/detector/detector_input.h"
#include "vc/detector/detector_output.h"
#include "vc/detector/rune_detector.h"
#include "vc/feature/feature_node.h"
#include "vc/feature/rune_combo.h"
#include "vc/feature/rune_group.h"
#include "vc/feature/rune_tracker.h"
#include "vc/feature/tracking_feature_node.h"

#include <chrono>
#include <memory>
#include <opencv2/core/utility.hpp>
#include <vector>

using namespace std;
using namespace cv;

namespace rmcs::identifier {

constexpr int kTargetSwitchConfirmFrames  = 5;
constexpr int kTargetSwitchCooldownFrames = 5;

struct TargetSelectionState {
    std::weak_ptr<RuneTracker> locked_tracker;
    std::weak_ptr<RuneTracker> pending_tracker;
    int pending_frames  = 0;
    int cooldown_frames = 0;

    void reset() {
        locked_tracker.reset();
        pending_tracker.reset();
        pending_frames  = 0;
        cooldown_frames = 0;
    }
};

struct BuffDetection::Impl {

    auto initialize([[maybe_unused]] const YAML::Node& yaml) noexcept
        -> std::expected<void, std::string> {
        return { };
    }

    shared_ptr<RuneTracker> buff_detect(const BuffDetectionFrame& frame) {
        const auto origin_mat = frame.image->details().mat;
        if (origin_mat.empty()) [[unlikely]] {
            return nullptr;
        }

        static auto rune_groups   = vector<FeatureNode_ptr> { };
        static auto rune_detector = RuneDetector::make_detector();
        static TargetSelectionState target_selection_state;
        DetectorInput input;
        DetectorOutput output;

        input.setImage(origin_mat);
        input.setGyroData(frame.gyro_data);
        input.setTick(frame.control_state.timestamp.time_since_epoch().count());
        input.setColor(frame.control_state.color);
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

        auto rune_tracker = selectTargetTracker(rune_group, target_selection_state);
        if (!rune_tracker) {
            return nullptr;
        }
        rune_tracker->setMotionMode(frame.control_state.shoot_mode == util::ShootMode::BUFF_LARGE);
        auto tracking_tracker =
            TrackingFeatureNode::cast(std::static_pointer_cast<FeatureNode>(rune_tracker));
        if (!tracking_tracker || tracking_tracker->getHistoryNodes().empty()) return nullptr;

        const auto combo = RuneCombo::cast(tracking_tracker->getHistoryNodes().front());
        if (!combo) return nullptr;

        const auto& child_features = combo->getChildFeatures();
        if (child_features.count(FeatureNode::ChildFeatureType::RUNE_TARGET) == 0) return nullptr;

        if (rune_tracker->isPredictionStable()) {
            return rune_tracker;
        } else {
            return nullptr;
        }
    }

private:
    RuneTracker_ptr selectTargetTracker(
        const RuneGroup_ptr& rune_group, TargetSelectionState& state) {
        RuneTracker_ptr best_tracker;
        size_t best_history_size = 0;
        RuneTracker_ptr locked_tracker;

        for (const auto& tracker : rune_group->getTrackers()) {
            auto tracking_tracker = TrackingFeatureNode::cast(tracker);
            if (!tracking_tracker || tracking_tracker->getHistoryNodes().empty()) continue;

            auto rune_tracker = RuneTracker::cast(tracker);
            if (!rune_tracker) continue;

            if (auto locked = state.locked_tracker.lock();
                locked && locked.get() == rune_tracker.get())
                locked_tracker = rune_tracker;

            auto combo = RuneCombo::cast(tracking_tracker->getHistoryNodes().front());
            if (!combo || combo->getRuneType() != RuneType::PENDING_STRUCK) continue;

            const size_t history_size = tracking_tracker->getHistoryNodes().size();
            if (!best_tracker || history_size > best_history_size) {
                best_tracker      = rune_tracker;
                best_history_size = history_size;
            }
        }

        if (!best_tracker) {
            state.pending_tracker.reset();
            state.pending_frames = 0;
            if (state.cooldown_frames > 0) --state.cooldown_frames;
            if (locked_tracker) return locked_tracker;
            state.locked_tracker.reset();
            return nullptr;
        }

        if (!locked_tracker) {
            state.locked_tracker = best_tracker;
            state.pending_tracker.reset();
            state.pending_frames  = 0;
            state.cooldown_frames = 0;
            return best_tracker;
        }

        if (locked_tracker.get() == best_tracker.get()) {
            state.pending_tracker.reset();
            state.pending_frames = 0;
            if (state.cooldown_frames > 0) --state.cooldown_frames;
            return locked_tracker;
        }

        if (state.cooldown_frames > 0) {
            --state.cooldown_frames;
            state.pending_tracker.reset();
            state.pending_frames = 0;
            return locked_tracker;
        }

        if (auto pending = state.pending_tracker.lock();
            pending && pending.get() == best_tracker.get())
            ++state.pending_frames;
        else {
            state.pending_tracker = best_tracker;
            state.pending_frames  = 1;
        }

        if (state.pending_frames >= kTargetSwitchConfirmFrames) {
            state.locked_tracker = best_tracker;
            state.pending_tracker.reset();
            state.pending_frames  = 0;
            state.cooldown_frames = kTargetSwitchCooldownFrames;
            return best_tracker;
        }

        return locked_tracker;
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
    -> std::optional<std::shared_ptr<RuneTracker>> {
    auto result = pimpl->buff_detect(frame);
    if (!result) {
        return std::nullopt;
    }
    return result;
}

}
