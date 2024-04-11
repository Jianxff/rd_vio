#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>


namespace rdvio {
const double GRAVITY_NOMINAL = 9.80665;

template <typename T> using map = Eigen::Map<T>;

template <typename T> using const_map = Eigen::Map<const T>;

inline constexpr size_t nil() { return size_t(-1); }

template <typename T>
struct compare; /*
    constexpr bool operator()(const T &a, const T &b) const;
*/

template <typename T> struct compare<T *> {
    constexpr bool operator()(const T *a, const T *b) const {
        return std::less<T>()(*a, *b);
    }
};



template <int Rows = Eigen::Dynamic, int Cols = Rows, bool UseRowMajor = false,
          typename T = double>
using matrix = typename std::conditional<
    Rows != 1 && Cols != 1,
    Eigen::Matrix<T, Rows, Cols,
                  UseRowMajor ? Eigen::RowMajor : Eigen::ColMajor>,
    Eigen::Matrix<T, Rows, Cols>>::type;

template <int Dimension = Eigen::Dynamic, bool RowVector = false,
          typename T = double>
using vector =
    typename std::conditional<RowVector, matrix<1, Dimension, false, T>,
                              matrix<Dimension, 1, false, T>>::type;

using quaternion = Eigen::Quaternion<double>;

struct ImuData {
    double t;
    vector<3> w;
    vector<3> a;
};

struct Pose {
    Pose() {
        q.setIdentity();
        p.setZero();
    }
    quaternion q;
    vector<3> p;
};

using OutputPose = Pose;
using uchar = unsigned char;

struct OutputState {
    double t;
    quaternion q;
    vector<3> p;
    vector<3> v;
    vector<3> bg;
    vector<3> ba;
};


class Config {
  public:
    virtual ~Config();

    virtual vector<2> camera_resolution() const = 0;
    virtual matrix<3> camera_intrinsic() const = 0;
    virtual vector<4> camera_distortion() const = 0;
    virtual quaternion camera_to_body_rotation() const = 0;
    virtual vector<3> camera_to_body_translation() const = 0;
    virtual size_t camera_distortion_flag() const = 0;
    virtual double camera_time_offset() const = 0;
    virtual quaternion imu_to_body_rotation() const = 0;
    virtual vector<3> imu_to_body_translation() const = 0;

    virtual matrix<2> keypoint_noise_cov() const = 0;
    virtual matrix<3> gyroscope_noise_cov() const = 0;
    virtual matrix<3> accelerometer_noise_cov() const = 0;
    virtual matrix<3> gyroscope_bias_noise_cov() const = 0;
    virtual matrix<3> accelerometer_bias_noise_cov() const = 0;

    virtual quaternion output_to_body_rotation() const;
    virtual vector<3> output_to_body_translation() const;

    virtual size_t sliding_window_size() const;
    virtual size_t sliding_window_subframe_size() const;
    virtual size_t sliding_window_force_keyframe_landmarks() const;
    virtual size_t sliding_window_tracker_frequent() const;

    virtual double feature_tracker_min_keypoint_distance() const;
    virtual size_t feature_tracker_max_keypoint_detection() const;
    virtual size_t feature_tracker_max_init_frames() const;
    virtual size_t feature_tracker_max_frames() const;
    virtual double feature_tracker_clahe_clip_limit() const;
    virtual size_t feature_tracker_clahe_width() const;
    virtual size_t feature_tracker_clahe_height() const;
    virtual bool feature_tracker_predict_keypoints() const;

    virtual size_t initializer_keyframe_num() const;
    virtual size_t initializer_keyframe_gap() const;
    virtual size_t initializer_min_matches() const;
    virtual double initializer_min_parallax() const;
    virtual size_t initializer_min_triangulation() const;
    virtual size_t initializer_min_landmarks() const;
    virtual bool initializer_refine_imu() const;

    virtual size_t solver_iteration_limit() const;
    virtual double solver_time_limit() const;

    virtual double rotation_misalignment_threshold() const;
    virtual double rotation_ransac_threshold() const;

    virtual int random() const;

    virtual bool parsac_flag() const;
    virtual double parsac_dynamic_probability() const;
    virtual double parsac_threshold() const;
    virtual double parsac_norm_scale() const;
    virtual size_t parsac_keyframe_check_size() const;

    void log_config() const;
};

class Image {
  public:
    double t;

    virtual uchar *get_rawdata() const = 0;
    virtual size_t width() const = 0;
    virtual size_t height() const = 0;

    virtual size_t level_num() const { return 0; }

    virtual double evaluate(const vector<2> &u, int level = 0) const = 0;
    virtual double evaluate(const vector<2> &u, vector<2> &ddu,
                            int level = 0) const = 0;

    virtual ~Image() = default;
    virtual void preprocess(double clipLimit, int width, int height) {}
    virtual void release_image_buffer() = 0;
    virtual void detect_keypoints(std::vector<vector<2>> &keypoints,
                                  size_t max_points = 0,
                                  double keypoint_distance = 0.5) const = 0;
    virtual void track_keypoints(const Image *next_image,
                                 const std::vector<vector<2>> &curr_keypoints,
                                 std::vector<vector<2>> &next_keypoints,
                                 std::vector<char> &result_status) const = 0;
};

enum SysState { SYS_INITIALIZING = 0, SYS_TRACKING, SYS_CRASH, SYS_UNKNOWN };


#define synchronized(obj_ptr)                                                  \
    if constexpr (auto local_synchronized_lock__ = (obj_ptr)->lock(); true)
    
} // namespace rdvio