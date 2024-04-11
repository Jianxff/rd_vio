#pragma once

#include <rdvio/types.h>
#include <rdvio/estimation/state.h>

namespace rdvio {
class Frontend;

class FeatureTracker {
  public:
    FeatureTracker(std::shared_ptr<Config> config);
    ~FeatureTracker();

    void set_frontend(std::shared_ptr<Frontend> frontend);

    bool empty() const { return frames.empty(); }

    void run();

    void track_frame(std::unique_ptr<Frame> frame);

    void mirror_map(Map *sliding_window_tracker_map);
    void synchronize_keymap(Map *sliding_window_tracker_map);
    // void mirror_keyframe(Map *sliding_window_tracker_map, size_t frame_id);
    void mirror_lastframe(Map *sliding_window_tracker_map);
    void attach_latest_frame(Frame *frame);
    // void manage_keymap();
    void solve_pnp();
    std::vector<Eigen::Vector2i> get_keypoints() const;

    std::optional<std::tuple<double, PoseState, MotionState>>
    get_latest_state() const;

    std::shared_ptr<Map> map;
    std::unique_ptr<Map> keymap;

  private:
    std::shared_ptr<Frontend> frontend;
    std::deque<std::unique_ptr<Frame>> frames;
    std::shared_ptr<Config> config;
    std::optional<std::tuple<double, PoseState, MotionState>> latest_state;
    mutable std::mutex latest_pose_mutex;
};

} // namespace rdvio

