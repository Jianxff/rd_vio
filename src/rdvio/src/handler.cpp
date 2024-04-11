#include <rdvio/types.h>
#include <rdvio/handler.h>
#include <rdvio/feature_tracker.h>
#include <rdvio/frontend.h>
#include <rdvio/estimation/solver.h>
#include <rdvio/geometry/lie_algebra.h>
#include <rdvio/geometry/stereo.h>
#include <rdvio/map/frame.h>
#include <rdvio/map/map.h>

namespace rdvio {

static void propagate_state(double &state_time, PoseState &state_pose,
                            MotionState &state_motion, double t,
                            const vector<3> &w, const vector<3> &a) {
    static const vector<3> gravity = {0, 0, -GRAVITY_NOMINAL};
    double dt = t - state_time;
    state_pose.p =
        state_pose.p + dt * state_motion.v +
        0.5 * dt * dt * (gravity + state_pose.q * (a - state_motion.ba));
    state_motion.v =
        state_motion.v + dt * (gravity + state_pose.q * (a - state_motion.ba));
    state_pose.q =
        (state_pose.q * expmap((w - state_motion.bg) * dt)).normalized();
    state_time = t;
}

Handler::Handler(std::shared_ptr<Config> config) : config(config) {
    Solver::init(config.get());

    feature_tracker = std::make_shared<FeatureTracker>(config);
    frontend = std::make_shared<Frontend>(feature_tracker, config);
    feature_tracker->set_frontend(frontend);
}

const Config *Handler::configurations() const { return config.get(); }

Pose Handler::track_gyroscope(const double &t, const double &x,
                                     const double &y, const double &z) {
    if (accelerometers.size() > 0) {
        if (t < accelerometers.front().t) {
            gyroscopes.clear();
        } else {
            while (accelerometers.size() > 0 && t >= accelerometers.front().t) {
                const auto &acc = accelerometers.front();
                double lambda =
                    (acc.t - gyroscopes[0].t) / (t - gyroscopes[0].t);
                vector<3> w = gyroscopes[0].w +
                              lambda * (vector<3>{x, y, z} - gyroscopes[0].w);
                track_imu({acc.t, w, acc.a});
                accelerometers.pop_front();
            }
            if (accelerometers.size() > 0) {
                while (gyroscopes.size() > 0 && gyroscopes.front().t < t) {
                    gyroscopes.pop_front();
                }
            }
        }
    }
    gyroscopes.emplace_back(GyroscopeData{t, {x, y, z}});
    return predict_pose(t);
}

Pose Handler::track_accelerometer(const double &t, const double &x,
                                         const double &y, const double &z) {
    if (gyroscopes.size() > 0 && t >= gyroscopes.front().t) {
        if (t > gyroscopes.back().t) {
            while (gyroscopes.size() > 1) {
                gyroscopes.pop_front();
            }
            // t > gyroscopes[0].t
            accelerometers.emplace_back(AccelerometerData{t, {x, y, z}});
        } else if (t == gyroscopes.back().t) {
            while (gyroscopes.size() > 1) {
                gyroscopes.pop_front();
            }
            track_imu({t, gyroscopes.front().w, {x, y, z}});
        } else {
            // pre-condition: gyroscopes.front().t <= t < gyroscopes.back().t
            // ==>  gyroscopes.size() >= 2
            while (t >= gyroscopes[1].t) {
                gyroscopes.pop_front();
            }
            // post-condition: t < gyroscopes[1].t
            double lambda =
                (t - gyroscopes[0].t) / (gyroscopes[1].t - gyroscopes[0].t);
            vector<3> w =
                gyroscopes[0].w + lambda * (gyroscopes[1].w - gyroscopes[0].w);
            track_imu({t, w, {x, y, z}});
        }
    }
    return predict_pose(t);
}

Pose Handler::track_camera(std::shared_ptr<Image> image) {
    std::unique_ptr<Frame> frame = std::make_unique<Frame>();
    frame->K = config->camera_intrinsic();
    frame->image = image;
    frame->sqrt_inv_cov = frame->K.block<2, 2>(0, 0);
    frame->sqrt_inv_cov(0, 0) /= ::sqrt(config->keypoint_noise_cov()(0, 0));
    frame->sqrt_inv_cov(1, 1) /= ::sqrt(config->keypoint_noise_cov()(1, 1));
    frame->camera.q_cs = config->camera_to_body_rotation();
    frame->camera.p_cs = config->camera_to_body_translation();
    frame->imu.q_cs = config->imu_to_body_rotation();
    frame->imu.p_cs = config->imu_to_body_translation();
    frame->preintegration.cov_a = config->accelerometer_noise_cov();
    frame->preintegration.cov_w = config->gyroscope_noise_cov();
    frame->preintegration.cov_ba = config->accelerometer_bias_noise_cov();
    frame->preintegration.cov_bg = config->gyroscope_bias_noise_cov();

    frames.emplace_back(std::move(frame));

    Pose outpose = predict_pose(image->t);
    std::unique_lock<std::mutex> lk(latest_mutex_);
    if (image->t > latest_timestamp_) {
        latest_pose_ = outpose;
        latest_timestamp_ = image->t;
    }
    return outpose;
}

void Handler::track_imu(const ImuData &imu) {
    frontal_imus.emplace_back(imu);
    imus.emplace_back(imu);
    while (imus.size() > 0 && frames.size() > 0) {
        if (imus.front().t <= frames.front()->image->t) {
            frames.front()->preintegration.data.push_back(imus.front());
            imus.pop_front();
        } else {
            feature_tracker->track_frame(std::move(frames.front()));
            frames.pop_front();
        }
    }
}

Pose Handler::predict_pose(const double &t) {
    Pose output_pose;
    if (auto maybe_state = feature_tracker->get_latest_state()) {
        auto [state_time, state_pose, state_motion] = maybe_state.value();

        while (!frontal_imus.empty() && frontal_imus.front().t <= state_time) {
            frontal_imus.pop_front();
        }
        for (const auto &imu : frontal_imus) {
            if (imu.t <= t) {
                propagate_state(state_time, state_pose, state_motion, imu.t,
                                imu.w, imu.a);
            }
        }
        output_pose.q = state_pose.q * config->output_to_body_rotation();
        output_pose.p =
            state_pose.p + state_pose.q * config->output_to_body_translation();
    } else {
        output_pose.q.coeffs().setZero();
        output_pose.p.setZero();
    }

    return output_pose;
}

SysState Handler::get_system_state() const {
    return frontend->get_system_state();
}

// for AR
std::tuple<double, Pose> Handler::get_latest_state() const {
    Pose output_pose;
    double timestamp;
    if (auto maybe_state = feature_tracker->get_latest_state()) {
        auto [state_time, state_pose, state_motion] = maybe_state.value();
        output_pose.q = state_pose.q;
        output_pose.p = state_pose.p;
        timestamp = state_time;
    } else {
        output_pose.q.coeffs().setZero();
        output_pose.p.setZero();
        timestamp = 0.0;
    }

    return {timestamp, output_pose};
}

std::tuple<double, Pose> Handler::get_latest_pose() {
    std::unique_lock<std::mutex> lk(latest_mutex_);
    return {latest_timestamp_, latest_pose_};
}

std::vector<Eigen::Vector3d> Handler::get_landmark() const {
    return frontend->get_landmark();
}

std::vector<Eigen::Vector2i> Handler::get_keypoints() const {
    return feature_tracker->get_keypoints();
}

} // namespace rdvio
