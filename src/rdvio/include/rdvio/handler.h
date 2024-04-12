#pragma once

#include <rdvio/types.h>
#include <mutex>

#ifdef USE_MULTI_THREADING
#include <thread>
#include <atomic>
#endif

namespace rdvio {

class Config;
class FeatureTracker;
class Frame;
class Frontend;
class Image;
class Map;
class Synchronizer;

class Handler {
    struct GyroscopeData {
        double t;
        vector<3> w;
    };
    struct AccelerometerData {
        double t;
        vector<3> a;
    };

  public:
    Handler(std::shared_ptr<Config> config);

    const Config *configurations() const;

    Pose track_gyroscope(const double &t, const double &x, const double &y,
                         const double &z);
    Pose track_accelerometer(const double &t, const double &x, const double &y,
                             const double &z);
    Pose track_camera(std::shared_ptr<Image> image);

    std::tuple<double, Pose> get_latest_state() const;
    std::tuple<double, Pose> get_latest_pose();

    std::shared_ptr<FeatureTracker> feature_tracker;
    std::shared_ptr<Frontend> frontend;

    SysState get_system_state() const;

    std::vector<Eigen::Vector3d> get_landmark() const;
    std::vector<Eigen::Vector2i> get_keypoints() const;

#ifdef USE_MULTI_THREADING
    void exit();
#endif

  private:
    void track_imu(const ImuData &imu);
    Pose predict_pose(const double &t);

    std::mutex latest_mutex_;
    double latest_timestamp_ = 0.0;
    Pose latest_pose_;

    std::deque<GyroscopeData> gyroscopes;
    std::deque<AccelerometerData> accelerometers;

    std::deque<ImuData> imus;
    std::deque<std::unique_ptr<Frame>> frames;
    std::deque<ImuData> frontal_imus;

    std::shared_ptr<Config> config;

#ifdef USE_MULTI_THREADING
    std::thread t_frontend_;
    std::thread t_feature_tracker_;
    std::atomic<bool> exit_ = false;
#endif
};

} // namespace rdvio
