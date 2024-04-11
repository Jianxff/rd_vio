#pragma once

#include <rdvio/handler.h>
#include <rdvio/util/debug.h>
#include <rdvio/extra/opencv_image.h>
#include <rdvio/extra/yaml_config.h>
#include <rdvio/map/frame.h>

namespace rdvio {

const Eigen::Matrix4d T_imu_to_cv = 
    (Eigen::Matrix4d() << 
        1, 0, 0, 0,
        0, 0, -1, 0,
        0, 1, 0, 0,
        0, 0, 0, 1
    ).finished();
const Eigen::Matrix3d R_imu_to_cv = 
    (Eigen::Matrix3d() << 
        1, 0, 0,
        0, 0, -1,
        0, 1, 0
    ).finished();

class Odometry{
public:
    Odometry(const std::string& calib, const std::string& config) {
        auto yaml_config = std::make_shared<extra::YamlConfig>(config, calib);
        handler_ = std::make_unique<Handler>(yaml_config);
        // set transform
        rotation_cam_to_body_ = yaml_config->camera_to_body_rotation().toRotationMatrix();
        translation_cam_to_body_ = yaml_config->camera_to_body_translation();
        transform_cam_to_body_ = _make_transform(
            yaml_config->camera_to_body_rotation(), 
            translation_cam_to_body_
        );
    }

    ~Odometry() = default;

    void addFrame(const double t, const cv::Mat& image) {
        auto image_ptr = std::make_shared<extra::OpenCvImage>();
        image_ptr->image = image.clone();
        image_ptr->raw = image.clone();
        image_ptr->t = t;
        handler_->track_camera(image_ptr);
    }

    void addMotion(const double t, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro) {
        handler_->track_gyroscope(t, gyro(0), gyro(1), gyro(2));
        handler_->track_accelerometer(t, acc(0), acc(1), acc(2));
    }

    const Eigen::Matrix4d transform_world_cam() {
        auto state = handler_->get_latest_state();
        Pose pose = std::get<1>(state);
        Eigen::Matrix4d Twb = _make_transform(pose.q, pose.p);
        Twb = T_imu_to_cv * Twb * transform_cam_to_body_;
        return Twb;
    }

    const int state() const {
        auto state = handler_->get_system_state();
        switch(state) {
            case SYS_INITIALIZING: return 0;
            case SYS_TRACKING: return 1;
            case SYS_CRASH: return 2;
            default: return 3;
        }
    }

    const std::vector<Eigen::Vector3d> local_map() const {
        auto points = std::move(handler_->get_landmark());
        for(auto& p : points) {
            p = R_imu_to_cv * p;
        }
        return points;
    }

    const std::vector<Eigen::Vector2i> keypoints() const {
        return handler_->get_keypoints();
    }

private:

    Eigen::Matrix4d _make_transform(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
        Eigen::Matrix3d R = q.toRotationMatrix();
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block<3, 3>(0, 0) = R;
        transform.block<3, 1>(0, 3) = t;
        return transform;
    }

    std::unique_ptr<Handler> handler_;
    Eigen::Matrix4d transform_cam_to_body_;
    Eigen::Matrix3d rotation_cam_to_body_;
    Eigen::Vector3d translation_cam_to_body_;
};



} // namespace rdvio