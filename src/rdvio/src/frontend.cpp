#include <iostream>
#include <rdvio/types.h>
#include <rdvio/feature_tracker.h>
#include <rdvio/frontend.h>
#include <rdvio/initializer.h>
#include <rdvio/sliding_window_tracker.h>
#include <rdvio/geometry/stereo.h>
#include <rdvio/map/frame.h>
#include <rdvio/map/map.h>
#include <rdvio/map/track.h>

namespace rdvio {

Frontend::Frontend(std::shared_ptr<FeatureTracker> feature_tracker,
                    std::shared_ptr<Config> config)
    : feature_tracker(feature_tracker), config(config) {
    initializer = std::make_unique<Initializer>(config);

    latest_state = {{}, nil(), {}, {}};
}

Frontend::~Frontend() = default;

bool Frontend::empty() const { return pending_frame_ids.empty(); }

void Frontend::run() {
    if(empty()) return;
    
    if (initializer) {
        size_t pending_frame_id = pending_frame_ids.front();
        pending_frame_ids.clear();
        synchronized(feature_tracker->map) {
            initializer->mirror_keyframe_map(feature_tracker->map.get(),
                                             pending_frame_id);
        }
        if ((sliding_window_tracker = initializer->initialize())) {
            sliding_window_tracker->feature_tracking_map =
                feature_tracker->map;
            std::unique_lock lk(latest_state_mutex);
            auto [t, pose, motion] = sliding_window_tracker->get_latest_state();
            latest_state = {t, pending_frame_id, pose, motion};
            lk.unlock();
            initializer.reset();
        }
    } else if (sliding_window_tracker) {
        size_t pending_frame_id = pending_frame_ids.front();
        pending_frame_ids.pop_front();
        synchronized(feature_tracker->map) {
            sliding_window_tracker->mirror_frame(
                feature_tracker->map.get(), pending_frame_id);
        }
        if (sliding_window_tracker->track()) {

            std::unique_lock lk(latest_state_mutex);
            auto [t, pose, motion] = sliding_window_tracker->get_latest_state();
            latest_state = {t, pending_frame_id, pose, motion};
            lk.unlock();
        } else {
            std::unique_lock lk(latest_state_mutex);
            latest_state = {{}, nil(), {}, {}};
            lk.unlock();
            initializer = std::make_unique<Initializer>(config);
            sliding_window_tracker.reset();
        }
    }
}

void Frontend::issue_frame(Frame *frame) {
    pending_frame_ids.push_back(frame->id());
#ifndef USE_MULTI_THREADING
    run();
#endif
}

std::tuple<double, size_t, PoseState, MotionState>
Frontend::get_latest_state() const {
    std::unique_lock lk(latest_state_mutex);
    return latest_state;
}

SysState Frontend::get_system_state() const {
    if (initializer) {
        return SysState::SYS_INITIALIZING;
    } else if (sliding_window_tracker) {
        return SysState::SYS_TRACKING;
    }
    return SysState::SYS_UNKNOWN;
}

std::vector<Eigen::Vector3d> Frontend::get_landmark() const {
    if (sliding_window_tracker) {
        return sliding_window_tracker->get_landmark();
    }
    return {};
}

} // namespace rdvio
