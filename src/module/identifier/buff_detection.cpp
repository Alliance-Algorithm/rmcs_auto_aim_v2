
#include "utility/image/image.hpp"
#include "vc/dataio/dataio.h"
#include "vc/detector/detector_input.h"
#include "vc/detector/detector_output.h"
#include "vc/detector/rune_detector.h"
#include "vc/feature/feature_node.h"
#include "vc/feature/rune_combo.h"
#include "vc/feature/rune_group.h"
#include "vc/feature/tracking_feature_node.h"
#include <memory>
#include <opencv2/core/utility.hpp>
#include <vector>

using namespace std;
using namespace cv;

void buff_detect(const std::unique_ptr<rmcs::Image>& image) {
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
        return;
    }

    rune_groups = output.getFeatureNodes();
    if (rune_groups.empty()) {
        return;
    }

    auto rune_group = RuneGroup::cast(rune_groups.front());
    if (rune_group == nullptr || rune_group->childFeatures().empty()) {
        return;
    }

    rune_group->setPredictFunc(
        [wg = std::weak_ptr<RuneGroup>(rune_group)](int64_t future_tick_ms) -> double {
            auto g = wg.lock();
            if (!g) return 0.0;

            const auto& ticks = g->getHistoryTicks(); // ms
            const auto& raw   = g->getRawDatas();     // deg(roll)

            if (ticks.size() < 2 || raw.size() < 2) {
                return raw.empty() ? 0.0 : raw.front();
            }

            const double t0 = static_cast<double>(ticks[0]); // 最新
            const double t1 = static_cast<double>(ticks[1]); // 次新
            const double a0 = static_cast<double>(raw[0]);   // 最新 roll (deg)
            const double a1 = static_cast<double>(raw[1]);   // 次新 roll (deg)

            const double dt = (t0 - t1); // ms
            if (dt <= 1e-6) return a0;

            // deg/ms
            const double w = (a0 - a1) / dt;

            const double df = static_cast<double>(future_tick_ms) - t0; // ms
            return a0 + w * df;
        });

    FeatureNode_cptr target_tracker = nullptr;
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
        return;
    }
}