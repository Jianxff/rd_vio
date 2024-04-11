#include <rdvio/types.h>
#include <rdvio/handler.h>
#include <rdvio/feature_tracker.h>
#include <rdvio/frontend.h>
#include <rdvio/estimation/solver.h>
#include <rdvio/geometry/stereo.h>
#include <rdvio/map/frame.h>
#include <rdvio/map/map.h>
#include <rdvio/map/track.h>
#include <rdvio/util/unique_timer.h>
#include <rdvio/util/debug.h>
namespace rdvio {

FeatureTracker::FeatureTracker(std::shared_ptr<Config> config)
    : config(config) {
    map = std::make_unique<Map>();
    keymap = std::make_unique<Map>();
}

FeatureTracker::~FeatureTracker() = default;

void FeatureTracker::set_frontend(std::shared_ptr<Frontend> frontend) {
    this->frontend = frontend;
}

void FeatureTracker::run() {
    if(empty()) return;

    std::unique_ptr<Frame> frame = std::move(frames.front());
    frames.pop_front();

    frame->image->preprocess(config->feature_tracker_clahe_clip_limit(),
                             config->feature_tracker_clahe_width(),
                             config->feature_tracker_clahe_height());

    auto [latest_optimized_time, latest_optimized_frame_id,
          latest_optimized_pose, latest_optimized_motion] =
        frontend->get_latest_state();
    bool is_initialized = latest_optimized_frame_id != nil();
    bool slidind_window_frame_tag =
        !is_initialized ||
        frame->id() % config->sliding_window_tracker_frequent() == 0;
    synchronized(map) {
        if (map->frame_num() > 0) {
            if (is_initialized) {
                size_t latest_optimized_frame_index =
                    map->frame_index_by_id(latest_optimized_frame_id);
                if (latest_optimized_frame_index != nil()) {
                    Frame *latest_optimized_frame =
                        map->get_frame(latest_optimized_frame_index);
                    latest_optimized_frame->pose = latest_optimized_pose;
                    latest_optimized_frame->motion = latest_optimized_motion;
                    for (size_t j = latest_optimized_frame_index + 1;
                         j < map->frame_num(); ++j) {
                        Frame *frame_i = map->get_frame(j - 1);
                        Frame *frame_j = map->get_frame(j);
                        frame_j->preintegration.integrate(
                            frame_j->image->t, frame_i->motion.bg,
                            frame_i->motion.ba, false, false);
                        frame_j->preintegration.predict(frame_i, frame_j);
                    }
                } else {
                    // TODO: unfortunately the frame has slided out, which means
                    // we are lost...
                    log_warning("SWT cannot catch up.");
                    std::unique_lock lk(latest_pose_mutex);
                    latest_state.reset();
                }
            }
            Frame *last_frame = map->get_frame(map->frame_num() - 1);
            if (!last_frame->preintegration.data.empty()) {
                if (frame->preintegration.data.empty() ||
                    (frame->preintegration.data.front().t -
                         last_frame->image->t >
                     1.0e-5)) {
                    ImuData imu = last_frame->preintegration.data.back();
                    imu.t = last_frame->image->t;
                    frame->preintegration.data.insert(
                        frame->preintegration.data.begin(), imu);
                }
            }
            frame->preintegration.integrate(
                frame->image->t, last_frame->motion.bg, last_frame->motion.ba,
                false, false);
            last_frame->track_keypoints(frame.get(), config.get());
            if (is_initialized) {
                frame->preintegration.predict(last_frame, frame.get());

                std::unique_lock lk(latest_pose_mutex);
                latest_state = {frame->image->t, frame->pose, frame->motion};
                lk.unlock();

            }
            last_frame->image->release_image_buffer();
        }

        if (slidind_window_frame_tag)
            frame->detect_keypoints(config.get());
        map->attach_frame(std::move(frame));

        while (map->frame_num() >
                   (is_initialized
                        ? config->feature_tracker_max_frames()
                        : config->feature_tracker_max_init_frames()) &&
               map->get_frame(0)->id() < latest_optimized_frame_id) {
            map->erase_frame(0);
        }
    }
    if (slidind_window_frame_tag)
        frontend->issue_frame(map->get_frame(map->frame_num() - 1));
}

void FeatureTracker::track_frame(std::unique_ptr<Frame> frame) {
    frames.emplace_back(std::move(frame));
    run();
}

std::optional<std::tuple<double, PoseState, MotionState>>
FeatureTracker::get_latest_state() const {
    std::unique_lock lk(latest_pose_mutex);
    return latest_state;
}

void FeatureTracker::synchronize_keymap(Map *sliding_window_tracker_map) {

    // clean keymap
    while (keymap->frame_num()) {
        keymap->erase_frame(0);
    }

    // mirror the latest SWT map to keymap
    mirror_map(sliding_window_tracker_map);

    // add last frame (include subframe) in swt map to keymap for track
    // associating
    mirror_lastframe(sliding_window_tracker_map);
}

void FeatureTracker::mirror_map(Map *sliding_window_tracker_map) {

    for (size_t index = 0; index < sliding_window_tracker_map->frame_num();
         ++index) {
        keymap->attach_frame(
            sliding_window_tracker_map->get_frame(index)->clone());
    }

    for (size_t j = 1; j < keymap->frame_num(); ++j) {
        Frame *old_frame_i = sliding_window_tracker_map->get_frame(j - 1);
        Frame *old_frame_j = sliding_window_tracker_map->get_frame(j);
        Frame *new_frame_i = keymap->get_frame(j - 1);
        Frame *new_frame_j = keymap->get_frame(j);
        for (size_t ki = 0; ki < old_frame_i->keypoint_num(); ++ki) {
            if (Track *track = old_frame_i->get_track(ki)) {
                if (size_t kj = track->get_keypoint_index(old_frame_j);
                    kj != nil()) {
                    Track *new_track = new_frame_i->get_track(ki, keymap.get());
                    new_track->add_keypoint(new_frame_j, kj);
                    new_track->landmark = track->landmark;
                    new_track->tag(TT_VALID) = track->tag(TT_VALID);
                    new_track->tag(TT_TRIANGULATED) =
                        track->tag(TT_TRIANGULATED);
                    new_track->tag(TT_FIX_INVD) = true;
                }
            }
        }
    }

    for (size_t index = 0; index < keymap->frame_num(); ++index) {
        Frame *keyframe = keymap->get_frame(index);
        keyframe->tag(FT_KEYFRAME) = true;
        keyframe->tag(FT_FIX_POSE) = true;
        keyframe->tag(FT_FIX_MOTION) = true;
    }
}

void FeatureTracker::mirror_lastframe(Map *sliding_window_tracker_map) {

    Frame *last_keyframe_i = keymap->get_frame(keymap->frame_num() - 1);
    Frame *last_keyframe_j = sliding_window_tracker_map->get_frame(
        sliding_window_tracker_map->frame_num() - 1);

    if (last_keyframe_j->subframes.empty()) // no subframes means this keyframe
                                            // has been existed in FT map
        return;

    Frame *last_subframe = last_keyframe_j->subframes.back().get();

    keymap->attach_frame(last_subframe->clone());

    Frame *new_keyframe = keymap->get_frame(keymap->frame_num() - 1);

    for (size_t ki = 0; ki < last_keyframe_j->keypoint_num(); ++ki) {
        if (Track *track = last_keyframe_j->get_track(ki)) {
            if (size_t kj = track->get_keypoint_index(last_subframe);
                kj != nil()) {
                last_keyframe_i->get_track(ki, keymap.get())
                    ->add_keypoint(new_keyframe, kj);
            }
        }
    }

    new_keyframe->tag(FT_KEYFRAME) = false;
    new_keyframe->tag(FT_FIX_POSE) = false;
    new_keyframe->tag(FT_FIX_MOTION) = false;
}

void FeatureTracker::attach_latest_frame(Frame *frame) {

    Frame *new_last_frame_i = keymap->get_frame(keymap->frame_num() - 1);
    size_t last_frame_index = map->frame_index_by_id(new_last_frame_i->id());

    size_t frame_index_i = map->frame_index_by_id(
        keymap->get_frame(keymap->frame_num() - 1)->id());
    size_t frame_index_j = map->frame_num() - 1;

    keymap->attach_frame(frame->clone());
    Frame *new_last_frame_j = keymap->get_frame(keymap->frame_num() - 1);

    if (last_frame_index != nil()) {
        Frame *old_last_frame_i = map->get_frame(last_frame_index);
        Frame *old_last_frame_j = frame;
        for (size_t ki = 0; ki < old_last_frame_i->keypoint_num(); ++ki) {
            if (Track *track = old_last_frame_i->get_track(ki)) {
                if (size_t kj = track->get_keypoint_index(old_last_frame_j);
                    kj != nil()) {
                    Track *track =
                        new_last_frame_i->get_track(ki, keymap.get());
                    track->add_keypoint(new_last_frame_j, kj);
                }
            }
        }
        new_last_frame_j->tag(FT_KEYFRAME) = false;
        new_last_frame_j->tag(FT_FIX_POSE) = false;
    } else {
        std::cout << "error: cannot find last frame id in FT map" << std::endl;
    }
}

void FeatureTracker::solve_pnp() {

    Frame *latest_frame = keymap->get_frame(keymap->frame_num() - 1);

    auto solver = Solver::create();

    solver->add_frame_states(latest_frame);

    for (size_t j = 0; j < latest_frame->keypoint_num(); ++j) {
        if (Track *track = latest_frame->get_track(j)) {
            if (track->all_tagged(TT_VALID, TT_TRIANGULATED)) {
                solver->put_factor(Solver::create_reprojection_prior_factor(
                    latest_frame, track));
            }
        }
    }
}


std::vector<Eigen::Vector2i> FeatureTracker::get_keypoints() const {
    std::vector<Eigen::Vector2i> keypoints;
    auto frame = map->get_frame(map->frame_num() - 1);
    keypoints.reserve(frame->keypoint_num());
    for (size_t i = 0; i < keymap->frame_num(); ++i) {
        if(Track* track = frame->get_track(i)) {
            keypoints.emplace_back(
                apply_k(frame->get_keypoint(i), frame->K).cast<int>()
            );
        }
    }
    return keypoints;
}

} // namespace rdvio
