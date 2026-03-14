
#include "utility/image/image.hpp"
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

shared_ptr<RuneTracker> buff_detect(const std::unique_ptr<rmcs::Image>& image) {
    static auto rune_groups   = vector<FeatureNode_ptr> { };
    static auto rune_detector = RuneDetector::make_detector();

    DetectorInput input;
    DetectorOutput output;

    input.setImage(image);
    input.setGyroData(GyroData { });
    input.setTick(cv::getTickCount());
    input.setColor(1); // TODO： 对接rmcs
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