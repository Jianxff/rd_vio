#include <sstream>
#include <rdvio/extra/yaml_config.h>
#include <yaml-cpp/yaml.h>

namespace rdvio::extra {

static YAML::Node find_node(const YAML::Node &root, const std::string &path,
                            bool mandatory = false) {
    std::stringstream ss(path);
    std::string child;
    YAML::Node node = root;
    while (std::getline(ss, child, '.')) {
        node.reset(node[child]);
    }
    if (!node) {
        if (mandatory) {
            throw YamlConfig::ConfigMissingException(path);
        }
    } else {
        node.SetTag(path);
    }
    return node;
}

static void require_vector(const YAML::Node &node, size_t n) {
    if (node.size() != n) {
        throw YamlConfig::TypeErrorException(node.Tag());
    }
    for (size_t i = 0; i < n; ++i) {
        if (!node[i].IsScalar()) {
            throw YamlConfig::TypeErrorException(node.Tag());
        }
    }
}

static void assign(bool &value, const YAML::Node &node) {
    if (!node.IsScalar()) {
        throw YamlConfig::TypeErrorException(node.Tag());
    }
    value = node.as<bool>();
}

static void assign(std::string &value, const YAML::Node &node) {
    if (!node.IsScalar()) {
        throw YamlConfig::TypeErrorException(node.Tag());
    }
    value = node.as<std::string>();
}

static void assign(size_t &value, const YAML::Node &node) {
    if (!node.IsScalar()) {
        throw YamlConfig::TypeErrorException(node.Tag());
    }
    value = node.as<size_t>();
}

static void assign(double &value, const YAML::Node &node) {
    if (!node.IsScalar()) {
        throw YamlConfig::TypeErrorException(node.Tag());
    }
    value = node.as<double>();
}

template <typename V>
static void assign_vector(V &vec, const YAML::Node &node) {
    require_vector(node, vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        vec[i] = node[i].as<double>();
    }
}

template <typename M>
static void assign_matrix(M &mat, const YAML::Node &node) {
    require_vector(node, mat.rows() * mat.cols());
    for (size_t i = 0; i < mat.rows(); ++i) {
        for (size_t j = 0; j < mat.cols(); ++j) {
            mat(i, j) = node[i * mat.cols() + j].template as<double>();
        }
    }
}

YamlConfig::YamlConfig(const std::string &slam_config_filename,
                       const std::string &device_config_filename) {
    m_output_to_body_rotation = Config::output_to_body_rotation();
    m_output_to_body_translation = Config::output_to_body_translation();
    m_sliding_window_size = Config::sliding_window_size();
    m_sliding_window_subframe_size = Config::sliding_window_subframe_size();
    m_sliding_window_tracker_frequent =
        Config::sliding_window_tracker_frequent();
    m_sliding_window_force_keyframe_landmarks =
        Config::sliding_window_force_keyframe_landmarks();
    m_feature_tracker_min_keypoint_distance =
        Config::feature_tracker_min_keypoint_distance();
    m_feature_tracker_max_keypoint_detection =
        Config::feature_tracker_max_keypoint_detection();
    m_feature_tracker_max_init_frames =
        Config::feature_tracker_max_init_frames();
    m_feature_tracker_max_frames = Config::feature_tracker_max_frames();
    m_feature_tracker_clahe_clip_limit =
        Config::feature_tracker_clahe_clip_limit();
    m_feature_tracker_clahe_width = Config::feature_tracker_clahe_width();
    m_feature_tracker_clahe_height = Config::feature_tracker_clahe_height();
    m_feature_tracker_predict_keypoints =
        Config::feature_tracker_predict_keypoints();
    m_initializer_keyframe_num = Config::initializer_keyframe_num();
    m_initializer_keyframe_gap = Config::initializer_keyframe_gap();
    m_initializer_min_matches = Config::initializer_min_matches();
    m_initializer_min_parallax = Config::initializer_min_parallax();
    m_initializer_min_triangulation = Config::initializer_min_triangulation();
    m_initializer_min_landmarks = Config::initializer_min_landmarks();
    m_initializer_refine_imu = Config::initializer_refine_imu();
    m_solver_iteration_limit = Config::solver_iteration_limit();
    m_solver_time_limit = Config::solver_time_limit();

    m_parsac_flag = Config::parsac_flag();
    m_parsac_dynamic_probability = Config::parsac_dynamic_probability();
    m_parsac_threshold = Config::parsac_threshold();
    m_parsac_norm_scale = Config::parsac_norm_scale();

    m_rotation_misalignment_threshold =
        Config::rotation_misalignment_threshold();
    m_rotation_ransac_threshold = Config::rotation_ransac_threshold();

    YAML::Node slam_config;
    YAML::Node device_config;
    try {
        slam_config = YAML::LoadFile(slam_config_filename);
    } catch (const YAML::ParserException &parse_error) {
        throw ParseException(parse_error.what());
    } catch (...) {
        throw LoadException(slam_config_filename);
    }
    try {
        device_config = YAML::LoadFile(device_config_filename);
    } catch (const YAML::ParserException &parse_error) {
        throw ParseException(parse_error.what());
    } catch (...) {
        throw LoadException(device_config_filename);
    }

    if (auto intrinsic = find_node(device_config, "cam0.intrinsics", true)) {
        require_vector(intrinsic, 4);
        m_camera_intrinsic.setIdentity();
        m_camera_intrinsic(0, 0) = intrinsic[0].as<double>();
        m_camera_intrinsic(1, 1) = intrinsic[1].as<double>();
        m_camera_intrinsic(0, 2) = intrinsic[2].as<double>();
        m_camera_intrinsic(1, 2) = intrinsic[3].as<double>();
    }

    if (auto node = find_node(device_config, "cam0.distortion", true)) {
        assign_vector(m_camera_distortion, node);
    }

    if (auto node =
            find_node(device_config, "cam0.camera_distortion_flag", true)) {
        assign(m_camera_distortion_flag, node);
    }

    if (auto node = find_node(device_config, "cam0.time_offset", true)) {
        assign(m_camera_time_offset, node);
    }

    if (auto node = find_node(device_config, "cam0.resolution", true)) {
        assign_vector(m_camera_resolution, node);
    }

    if (auto node = find_node(device_config, "cam0.extrinsic.q_bc", true)) {
        assign_vector(m_camera_to_body_rotation.coeffs(), node);
    }

    if (auto node = find_node(device_config, "cam0.extrinsic.p_bc", true)) {
        assign_vector(m_camera_to_body_translation, node);
    }

    if (auto node = find_node(device_config, "cam0.noise", true)) {
        assign_matrix(m_keypoint_noise_cov, node);
    }

    if (auto node = find_node(device_config, "imu.extrinsic.q_bi", true)) {
        assign_vector(m_imu_to_body_rotation.coeffs(), node);
    }

    if (auto node = find_node(device_config, "imu.extrinsic.p_bi", true)) {
        assign_vector(m_imu_to_body_translation, node);
    }

    if (auto node = find_node(device_config, "imu.noise.cov_g", true)) {
        assign_matrix(m_gyroscope_noise_cov, node);
    }

    if (auto node = find_node(device_config, "imu.noise.cov_a", true)) {
        assign_matrix(m_accelerometer_noise_cov, node);
    }

    if (auto node = find_node(device_config, "imu.noise.cov_bg", true)) {
        assign_matrix(m_gyroscope_bias_noise_cov, node);
    }

    if (auto node = find_node(device_config, "imu.noise.cov_ba", true)) {
        assign_matrix(m_accelerometer_bias_noise_cov, node);
    }

    if (auto node = find_node(slam_config, "output.q_bo", false)) {
        assign_vector(m_output_to_body_rotation.coeffs(), node);
    }

    if (auto node = find_node(slam_config, "output.p_bo", false)) {
        assign_vector(m_output_to_body_translation, node);
    }

    if (auto node = find_node(slam_config, "sliding_window.size", false)) {
        assign(m_sliding_window_size, node);
    }

    if (auto node =
            find_node(slam_config, "sliding_window.subframe_size", false)) {
        assign(m_sliding_window_subframe_size, node);
    }

    if (auto node =
            find_node(slam_config, "sliding_window.tracker_frequent", false)) {
        assign(m_sliding_window_tracker_frequent, node);
    }

    if (auto node = find_node(
            slam_config, "sliding_window.force_keyframe_landmarks", false)) {
        assign(m_sliding_window_force_keyframe_landmarks, node);
    }

    if (auto node = find_node(slam_config,
                              "feature_tracker.min_keypoint_distance", false)) {
        assign(m_feature_tracker_min_keypoint_distance, node);
    }

    if (auto node = find_node(
            slam_config, "feature_tracker.max_keypoint_detection", false)) {
        assign(m_feature_tracker_max_keypoint_detection, node);
    }

    if (auto node =
            find_node(slam_config, "feature_tracker.max_init_frames", false)) {
        assign(m_feature_tracker_max_init_frames, node);
    }

    if (auto node =
            find_node(slam_config, "feature_tracker.max_frames", false)) {
        assign(m_feature_tracker_max_frames, node);
    }

    if (auto node =
            find_node(slam_config, "feature_tracker.clahe_clip_limit", false)) {
        assign(m_feature_tracker_clahe_clip_limit, node);
    }

    if (auto node =
            find_node(slam_config, "feature_tracker.clahe_width", false)) {
        assign(m_feature_tracker_clahe_width, node);
    }

    if (auto node =
            find_node(slam_config, "feature_tracker.clahe_height", false)) {
        assign(m_feature_tracker_clahe_height, node);
    }

    if (auto node = find_node(slam_config, "feature_tracker.predict_keypoints",
                              false)) {
        assign(m_feature_tracker_predict_keypoints, node);
    }

    if (auto node = find_node(slam_config, "initializer.keyframe_num", false)) {
        assign(m_initializer_keyframe_num, node);
    }

    if (auto node = find_node(slam_config, "initializer.keyframe_gap", false)) {
        assign(m_initializer_keyframe_gap, node);
    }

    if (auto node = find_node(slam_config, "initializer.min_matches", false)) {
        assign(m_initializer_min_matches, node);
    }

    if (auto node = find_node(slam_config, "initializer.min_parallax", false)) {
        assign(m_initializer_min_parallax, node);
    }

    if (auto node =
            find_node(slam_config, "initializer.min_triangulation", false)) {
        assign(m_initializer_min_triangulation, node);
    }

    if (auto node =
            find_node(slam_config, "initializer.min_landmarks", false)) {
        assign(m_initializer_min_landmarks, node);
    }

    if (auto node = find_node(slam_config, "initializer.refine_imu", false)) {
        assign(m_initializer_refine_imu, node);
    }

    if (auto node = find_node(slam_config, "solver.iteration_limit", false)) {
        assign(m_solver_iteration_limit, node);
    }

    if (auto node = find_node(slam_config, "solver.time_limit", false)) {
        assign(m_solver_time_limit, node);
    }

    if (auto node = find_node(slam_config, "parsac.parsac_flag", false)) {
        assign(m_parsac_flag, node);
    }

    if (auto node =
            find_node(slam_config, "parsac.dynamic_probability", false)) {
        assign(m_parsac_dynamic_probability, node);
    }

    if (auto node = find_node(slam_config, "parsac.threshold", false)) {
        assign(m_parsac_threshold, node);
    }

    if (auto node = find_node(slam_config, "parsac.norm_scale", false)) {
        assign(m_parsac_norm_scale, node);
    }

    if (auto node =
            find_node(slam_config, "parsac.keyframe_check_size", false)) {
        assign(m_parsac_keyframe_check_size, node);
    }

    if (auto node =
            find_node(slam_config, "rotation.misalignment_threshold", false)) {
        assign(m_rotation_misalignment_threshold, node);
    }

    if (auto node =
            find_node(slam_config, "rotation.ransac_threshold", false)) {
        assign(m_rotation_ransac_threshold, node);
    }
}

YamlConfig::~YamlConfig() = default;

matrix<3> YamlConfig::camera_intrinsic() const { return m_camera_intrinsic; }

vector<4> YamlConfig::camera_distortion() const { return m_camera_distortion; }

size_t YamlConfig::camera_distortion_flag() const {
    return m_camera_distortion_flag;
}

double YamlConfig::camera_time_offset() const { return m_camera_time_offset; }

vector<2> YamlConfig::camera_resolution() const { return m_camera_resolution; }

quaternion YamlConfig::camera_to_body_rotation() const {
    return m_camera_to_body_rotation;
}

vector<3> YamlConfig::camera_to_body_translation() const {
    return m_camera_to_body_translation;
}

quaternion YamlConfig::imu_to_body_rotation() const {
    return m_imu_to_body_rotation;
}

vector<3> YamlConfig::imu_to_body_translation() const {
    return m_imu_to_body_translation;
}

matrix<2> YamlConfig::keypoint_noise_cov() const {
    return m_keypoint_noise_cov;
}

matrix<3> YamlConfig::gyroscope_noise_cov() const {
    return m_gyroscope_noise_cov;
}

matrix<3> YamlConfig::accelerometer_noise_cov() const {
    return m_accelerometer_noise_cov;
}

matrix<3> YamlConfig::gyroscope_bias_noise_cov() const {
    return m_gyroscope_bias_noise_cov;
}

matrix<3> YamlConfig::accelerometer_bias_noise_cov() const {
    return m_accelerometer_bias_noise_cov;
}

quaternion YamlConfig::output_to_body_rotation() const {
    return m_output_to_body_rotation;
}

vector<3> YamlConfig::output_to_body_translation() const {
    return m_output_to_body_translation;
}

size_t YamlConfig::sliding_window_size() const { return m_sliding_window_size; }

size_t YamlConfig::sliding_window_subframe_size() const {
    return m_sliding_window_subframe_size;
}

size_t YamlConfig::sliding_window_tracker_frequent() const {
    return m_sliding_window_tracker_frequent;
}

size_t YamlConfig::sliding_window_force_keyframe_landmarks() const {
    return m_sliding_window_force_keyframe_landmarks;
}

double YamlConfig::feature_tracker_min_keypoint_distance() const {
    return m_feature_tracker_min_keypoint_distance;
}

size_t YamlConfig::feature_tracker_max_keypoint_detection() const {
    return m_feature_tracker_max_keypoint_detection;
}

size_t YamlConfig::feature_tracker_max_init_frames() const {
    return m_feature_tracker_max_init_frames;
}

size_t YamlConfig::feature_tracker_max_frames() const {
    return m_feature_tracker_max_frames;
}

double YamlConfig::feature_tracker_clahe_clip_limit() const {
    return m_feature_tracker_clahe_clip_limit;
}

size_t YamlConfig::feature_tracker_clahe_width() const {
    return m_feature_tracker_clahe_width;
}

size_t YamlConfig::feature_tracker_clahe_height() const {
    return m_feature_tracker_clahe_height;
}

bool YamlConfig::feature_tracker_predict_keypoints() const {
    return m_feature_tracker_predict_keypoints;
}

size_t YamlConfig::initializer_keyframe_num() const {
    return m_initializer_keyframe_num;
}

size_t YamlConfig::initializer_keyframe_gap() const {
    return m_initializer_keyframe_gap;
}

size_t YamlConfig::initializer_min_matches() const {
    return m_initializer_min_matches;
}

double YamlConfig::initializer_min_parallax() const {
    return m_initializer_min_parallax;
}

size_t YamlConfig::initializer_min_triangulation() const {
    return m_initializer_min_triangulation;
}

size_t YamlConfig::initializer_min_landmarks() const {
    return m_initializer_min_landmarks;
}

bool YamlConfig::initializer_refine_imu() const {
    return m_initializer_refine_imu;
}

size_t YamlConfig::solver_iteration_limit() const {
    return m_solver_iteration_limit;
}

double YamlConfig::solver_time_limit() const { return m_solver_time_limit; }

bool YamlConfig::parsac_flag() const { return m_parsac_flag; }

double YamlConfig::parsac_dynamic_probability() const {
    return m_parsac_dynamic_probability;
}

double YamlConfig::parsac_threshold() const { return m_parsac_threshold; }

double YamlConfig::parsac_norm_scale() const { return m_parsac_norm_scale; }

size_t YamlConfig::parsac_keyframe_check_size() const {
    return m_parsac_keyframe_check_size;
}

double YamlConfig::rotation_misalignment_threshold() const {
    return m_rotation_misalignment_threshold;
}

double YamlConfig::rotation_ransac_threshold() const {
    return m_rotation_ransac_threshold;
}

} // namespace rdvio::extra
