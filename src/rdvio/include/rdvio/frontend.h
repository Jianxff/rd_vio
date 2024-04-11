#pragma once

#include <rdvio/types.h>
#include <rdvio/estimation/state.h>

namespace rdvio {

class Config;
class Frame;
class Initializer;
class FeatureTracker;
class SlidingWindowTracker;

class Frontend {
  public:
    Frontend(std::shared_ptr<FeatureTracker> feature_tracker, std::shared_ptr<Config> config);
    ~Frontend();

    bool empty() const;
    void run();

    void issue_frame(Frame *frame);

    std::tuple<double, size_t, PoseState, MotionState> get_latest_state() const;
    SysState get_system_state() const;
    std::vector<Eigen::Vector3d> get_landmark() const;

  private:
    std::deque<size_t> pending_frame_ids;
    
    std::shared_ptr<FeatureTracker> feature_tracker;
    std::shared_ptr<Config> config;
    std::unique_ptr<Initializer> initializer;
    std::unique_ptr<SlidingWindowTracker> sliding_window_tracker;

    std::tuple<double, size_t, PoseState, MotionState> latest_state;
    mutable std::mutex latest_state_mutex;
};

} // namespace rdvio
