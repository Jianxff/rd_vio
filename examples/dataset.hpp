#pragma once

#include <vector>
#include <unordered_map>
#include <fstream>
#include <filesystem>
#include <memory>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include <yaml-cpp/yaml.h>

namespace dataset
{

namespace fs = std::filesystem;

static bool __log_d__ = false;
/// @brief set logger flag
void set_logger(const bool log) { __log_d__ = log; }

/**
 * @brief Timestamp structure
 * @note [Convert] use sec(), ms(), us(), ns() to get converted data
 * @note [Template] call with template to get different data type
*/
struct timestamp_t {
    // basic nano second
    const uint64_t ns;

    /// @brief default constructor
    timestamp_t(uint64_t t) : ns(t) {}

    /// @brief from second
    template <typename T>
    static timestamp_t from_second(const double& t) 
        { return timestamp_t(static_cast<uint64_t>(static_cast<double>(t) * 1e9)); }

    /// @brief from microsecond
    template <typename T>
    static timestamp_t from_microsecond(const double& t) 
        { return timestamp_t(static_cast<uint64_t>(static_cast<double>(t) * 1e6)); }

    /// @brief from millisecond
    template <typename T>
    static timestamp_t from_millisecond(const T t) 
        { return timestamp_t(static_cast<uint64_t>(static_cast<double>(t) * 1e3)); }

    /// @brief from nanosecond
    static timestamp_t from_nanosecond(const uint64_t t) 
        { return timestamp_t(static_cast<uint64_t>(t)); }

    /// @brief convert to second
    template <typename T = double>
    const T sec() const { return static_cast<T>(__div__(1e9)); }

    /// @brief convert to microsecond
    template <typename T = double>
    const T ms() const { return static_cast<T>(__div__(1e6)); }

    /// @brief convert to millisecond
    template <typename T = double>
    const T us() const { return static_cast<T>(__div__(1e3)); }

    /// @brief convert to nanosecond
    template <typename T = uint64_t>
    const T nano() const { return static_cast<T>(ns); }

    /// @brief inline division
    inline double __div__(const double& d) const { return static_cast<double>(ns) / d; }
};


/**
 * @brief Motion data structure (accelerometer or gyroscope)
 * @note [Format] data stored in x, y, z format. 
 * @note [Convert] data can be converted to Eigen format using the eigen() function.
*/
struct motion_t {
    double x, y, z;

    /// @brief convert to Eigen
    template <typename T = double>
    Eigen::Vector3<T> eigen() const {
        return Eigen::Vector3<T>(x, y, z);
    }
};


/**
 * @brief instant clip for data
 * @param ts timestamp
 * @param cam0 main frame data
 * @param cam1 extra frame data
 * @param depth depth frame data
 * @param accel imu acceleration
 * @param gyro imu gyroscope
 * @note The frame images are stored in OpenCV Mat format.
*/
struct dataclip_t {
    /// @brief default constructor
    dataclip_t(const uint64_t ns) : timestamp(ns) {}
    /// @brief default destructor
    ~dataclip_t() {
        if(!cam0.empty()) cam0.release();
        if(!cam1.empty()) cam1.release();
        if(!depth.empty()) depth.release();
    }
    // timestamp
    const timestamp_t timestamp;
    // main frame data
    cv::Mat cam0;
    // extra frame data
    cv::Mat cam1, depth;
    // imu accelerometer and gyroscope
    motion_t accelerometer, gyroscope;
    // flags
    enum {CAM0 = 0, CAM1 = 1, DEPTH = 2, ACC = 3, GYRO = 4};
    bool available[5] = {false, false, false, false, false};
    /**
     * @brief set and unset data availability
    */
    void set(const int idx) { available[idx] = true; }
    void unset(const int idx) { available[idx] = false; }
    /**
     * @brief if dataclip valid
     * @note [Assert] one of cam0, cam1, depth, motion data should be available
    */
    bool valid() { return available[0] && available[1] && available[2] && available[3] && available[4]; }
    bool has_cam0() { return available[CAM0]; }
    bool has_cam1() { return available[CAM1]; }
    bool has_depth() { return available[DEPTH]; }
    bool has_acc() { return available[ACC]; }
    bool has_gyro() { return available[GYRO]; }
    bool has_motion() { return available[ACC] && available[GYRO]; }
    template <typename T = double>
    Eigen::Vector3<T> acc() { return accelerometer.eigen<T>(); }
    template <typename T = double>
    Eigen::Vector3<T> gyro() { return gyroscope.eigen<T>(); }
};


/**
 * @brief Intrinsic camera parameters
 * @note [Format] fx, fy, cx, cy
 * @note [Convert] data can be converted to Eigen format using the eigen() function.
 * @note [Convert] data can be converted to cv::Mat format using the cv() function.
*/
struct intrinsic_t {
    // camera intrinsict
    float fx, fy, cx, cy;
    /// @brief convert to Eigen
    template <typename T = float>
    Eigen::Matrix<T, 3, 3> eigen() const {
        Eigen::Matrix<T, 3, 3> K;
        K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
        return K;
    }
    /// @brief convert to cv::Mat
    template <typename T = float>
    cv::Mat cv() const {
        return (cv::Mat_<T>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    }
};


/**
 * @brief Distortion parameters
 * @note [Format] k1, k2, p1, p2, k3
 * @note [Convert] data can be converted to Eigen format using the eigen() function.
 * @note [Convert] data can be converted to cv::Mat format using the cv() function.
*/
struct distortion_t {
    // distortion parameters
    double k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;
    /// @brief convert to Eigen
    template <typename T = double>
    Eigen::VectorX<T> eigen(bool k3 = false) const {
        if(k3) {
            Eigen::Matrix<T, 5, 1> D; D << k1, k2, p1, p2, k3;
            return D;
        }
        Eigen::Matrix<T, 4, 1> D; D << k1, k2, p1, p2;
        return D;
    }
    /// @brief convert to cv::Mat
    template <typename T = double>
    cv::Mat cv(bool k3 = false) const {
        if(k3) return (cv::Mat_<T>(5, 1) << k1, k2, p1, p2, k3);
        return (cv::Mat_<T>(4, 1) << k1, k2, p1, p2);
    }
};


/**
 * @brief Load matrix from yaml
*/
template <typename T = float>
static Eigen::MatrixX<T> matrix_from_yaml(const YAML::Node& node) {
    const int rows = node["rows"].as<int>(1);
    const int cols = node["cols"].as<int>(1);
    const std::vector<T> raw_data = node["data"].as<std::vector<T>>();
    // convert to eigen
    Eigen::MatrixX<T> M = Eigen::Map<
        const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    >(raw_data.data(), rows, cols);
    return M;
}


/**
 * @brief Load vector from yaml
*/
template <typename T = float>
static Eigen::VectorX<T> vector_from_yaml(const YAML::Node& node) {
    const std::vector<T> raw_data = node.as<std::vector<T>>();
    const int rows = raw_data.size();
    // convert to eigen
    Eigen::VectorX<T> V = Eigen::Map<const Eigen::VectorX<T>>(raw_data.data(), rows);
    return V;
}


/**
 * @brief Undistort image using intrinsic and distortion parameters
 * @param img input image
 * @param K intrinsic parameters
 * @param D distortion parameters
 * @note [Return] return undistorted image
*/
static cv::Mat undistort_image(const cv::Mat& img, const intrinsic_t& K, const distortion_t& D) {
    cv::Mat undistorted;
    cv::undistort(img, undistorted, K.cv<float>(), D.cv<float>());
    return undistorted;
}


/**
 * @brief Predict intrinsic parameters
 * @param width image width
 * @param height image height
 * @note [Return] return predicted intrinsic parameters
*/
intrinsic_t predict_intrinsic(const int width, const int height) {
    float focal = static_cast<float>(std::max(width, height)) * 1.2;
    return intrinsic_t {
        .fx = focal,
        .fy = focal,
        .cx = static_cast<float>(width / 2),
        .cy = static_cast<float>(height / 2)
    };
}


/**
 * @brief Simple csv reader
*/
class csvReader {
public:
    /**
     * @brief Constructor
     * @param filepath csv file path
     * @note [Structure] data must be separated by comma ',' and comment with '#'
    */
    csvReader(const std::string& filepath) : ifs_(filepath) {
        if(!ifs_.is_open()) {
            throw std::runtime_error("cannot open file: " + filepath);
        }
        ss_cache_ = std::stringstream("");
    }

    /// @brief whether end of last data
    bool isEnd() { return ss_end() && (ifs_.peek() == EOF || ifs_.eof()); }

    /**
     * @brief get next data
     * @param n number of data to read, 1 by default
     * @note [Return] return vector of data
    */
    template <typename T>
    std::vector<T> next(size_t n = 1) {
        std::vector<T> data;
        for(size_t i = 0; i < n; i++) {
            if(isEnd()) break;
            T d = _read_next<T>();
            data.push_back(d);
        }
        return data;
    }

    /**
     * @brief ignore next data
     * @param n number of data to ignore, 1 by default
    */
    void ignore(size_t n = 1) {
        for(size_t i = 0; i < n; i++) {
            if(isEnd()) break;
            _read_next<std::string>();
        }
    }

    /// @brief close file
    void close() {
        ifs_.close();
    }

private:
    /// @brief read next single data
    template <typename T>
    T _read_next() {
        if( ss_end() ) {
            bool available = false;
            std::string line = "";
            while(!ifs_.eof()) {
                std::getline(ifs_, line);
                available = format_and_check(line);
                if (available) break;
            }
            if(!available) return T();
            else ss_cache_ = std::stringstream(line);
        }
        
        T data;
        ss_cache_ >> data;
        if(ss_cache_.fail()) {
            throw std::runtime_error("failed to read data of type: " + std::string(typeid(data).name()) + " from line: " + ss_cache_.str());
        }
        return data;
    }

    /// @brief format ane check line
    bool format_and_check(std::string& line) {
        if(line.size() == 0) return false;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::replace(line.begin(), line.end(), '\t', ' ');
        size_t first = line.find_first_not_of(' '), last = line.find_last_not_of(' ');
        line = line.substr(first, last - first + 1);
        return (!line.empty() && line.front() != '#');
    }

    bool ss_end() { 
        if(ss_cache_.eof() || ss_cache_.str().empty()) return true;
        char c = ss_cache_.peek();
        return c == '\0' || c == '\n' || c == '\r' || c == EOF;
    }

    std::ifstream ifs_;
    std::stringstream ss_cache_;

}; // class CSV


/**
 * @brief Base class
 * @note The base class is used to define the basic structure of the dataset.
*/
class Base {
public:
    /**
     * @brief Constructor
     * @param base_dir base path or directory of the dataset
    */
    Base(const std::string& base_dir) : base_dir_(fs::canonical(fs::absolute(base_dir))) {
        if(!fs::exists(base_dir_))
            throw std::runtime_error("file or directory not found: " + base_dir_.string());
    }

    /// @brief Destructor
    virtual ~Base() {}

    /// @brief get next data
    virtual std::shared_ptr<dataclip_t> next() = 0;

    /// @brief get previous data
    virtual std::shared_ptr<dataclip_t> prev() {
        throw std::runtime_error("Not implemented: prev()");
    }

    /// @brief get data by timestamp
    virtual std::shared_ptr<dataclip_t> at(const uint64_t ns) {
        throw std::runtime_error("Not implemented: at(time)");
    };

    /// @brief reset data
    virtual void reset() { 
        throw std::runtime_error("Not implemented: reset()");
    };
    
    /// @brief get current timestamp
    virtual timestamp_t current_ts() {
        if(data_pt_ >= data_.size()) return timestamp_t(-1);
        return data_[data_pt_]->timestamp;
    }

    /// @brief get seq length
    virtual size_t len() { return data_.size(); }

protected:
    /// @brief get and create data by index
    std::shared_ptr<dataclip_t> get_(const uint64_t ns, const bool create_if_empty = false) {
        if(data_map_.count(ns) > 0) return data_map_[ns];
        else if(create_if_empty) {
            const auto it = std::make_shared<dataclip_t>(ns);
            data_.push_back(it);
            data_map_[ns] = it;
            return data_map_[ns];
        }
        return nullptr;
    }

    /// @brief delete by index
    void del_(const uint64_t ns) {
        if(data_map_.count(ns) == 0) return;
        data_map_.erase(ns);
        data_.erase(std::remove_if(data_.begin(), data_.end(), 
            [ns](const std::shared_ptr<dataclip_t>& d) { return d->timestamp.ns == ns; }), data_.end());
    }

    /// @brief free last
    virtual void _free_last() {
        if(data_pt_ == 0) return;
        const auto ts = data_[data_pt_ - 1]->timestamp.ns;
        data_map_.erase(ts);
        data_[data_pt_ - 1].reset();
    }

    /// @brief order data by timestamp
    void _order_by_time() {
        std::sort(data_.begin(), data_.end(), 
            [](const std::shared_ptr<dataclip_t>& a, const std::shared_ptr<dataclip_t>& b) {
                return a->timestamp.ns < b->timestamp.ns;
            });
    }

    /// @brief base directory
    const fs::path base_dir_;

    /// @brief data list
    std::vector<std::shared_ptr<dataclip_t>> data_;
    std::unordered_map<uint64_t, std::shared_ptr<dataclip_t>> data_map_;

    /// @brief data pointer
    size_t data_pt_ = 0;

}; // class Base



/**
 * @brief EuRoC dataset
 * @note [Get] Sequentially get data using next() function
*/
class EuRoC : public Base {
public:
    /**
     * @brief Constructor
     * @param mav_dir mav directory of EuRoC dataset
     * @param motion_d load imu motion data (optional, true)
     * @param stereo_d load stereo cam1 data (optional, true)
     * @param auto_free auto free previous data (optional, false)
     * @param pre_load pre load all images (optional, false)
     * @note The EuRoC dataset is stored in a directory with the following structure:
    */
    EuRoC(const std::string& mav_dir, const bool motion_d = true, const bool stereo_d = true, const bool auto_free = false, const bool pre_load = false) 
     : Base(mav_dir), motion_enabled_(motion_d), stereo_enabled_(stereo_d), auto_free_enabled_(auto_free)
    {
        if(__log_d__) {
            std::cout << "[dataset] load euroc dataset from " << mav_dir << std::endl;
            std::cout << "[euroc] imu motion " << (motion_d ? "enabled" : "disabled") << std::endl;
            std::cout << "[euroc] stereo " << (stereo_d ? "enabled" : "disabled") << std::endl;
        }

        _load_camera_data(stereo_enabled_);
        if(motion_enabled_) _load_motion_data();
        if(pre_load) _pre_load();
        _order_by_time();
    }

    /**
     * @brief get next dataclip
     * @note [Return] return nullptr if no next data available
    */
    std::shared_ptr<dataclip_t> next() override {
        if(data_pt_ >= data_.size()) return nullptr;
        auto data_at_pt = data_[data_pt_];
        // read for camera data
        const uint64_t ts = data_at_pt->timestamp.ns;
        // check avaliable for cam0 and load status
        if(data_at_pt->has_cam0() && data_at_pt->cam0.empty()) {
            data_at_pt->cam0 = _read_image(ts, 0);
        }
        // check avaliable for cam1 (stereo) and load status
        if(stereo_enabled_ && data_at_pt->available[dataclip_t::CAM1] && data_at_pt->cam1.empty()) {
            data_at_pt->cam1 = _read_image(ts, 1);
        }

        if(__log_d__) std::cout << "[euroc] " << (data_pt_ + 1) << " / " << data_.size() << " at " << ts << std::endl;

        if(auto_free_enabled_) _free_last();

        return data_[data_pt_++];
    }

    /// @brief get camera rate
    float cam_rate() { return cam_rate_hz_; }

    /// @brief get imu rate
    float imu_rate() { return imu_rate_hz_; }

private:
    /// @brief load camera sensor calib and frame
    void _load_camera_data(bool stereo) {
        if(__log_d__) std::cout << "[euroc] loading camera data" << std::endl;

        for(int i = 0; i <= static_cast<int>(stereo); ++i) {
            fs::path cam_csv = base_dir_ / ("cam" + std::to_string(i)) / "data.csv";
            fs::path cam_data = base_dir_ / ("cam" + std::to_string(i)) / "data";
            fs::path cam_yaml = base_dir_ / ("cam" + std::to_string(i)) / "sensor.yaml";
            auto csv = csvReader(cam_csv.string());
            
            size_t cnt = 0;
            while(!csv.isEnd()) {
                auto ts = csv.next<uint64_t>()[0]; csv.ignore();
                auto data = get_(ts, true);
                data->set(i);
                ++cnt;
            }

            if(__log_d__) std::cout << "[euroc] loaded " << cnt << " frames for cam" << i << std::endl;

            // read yaml
            YAML::Node node = YAML::LoadFile(cam_yaml.string());
            int width = node["resolution"][0].as<int>();
            int height = node["resolution"][1].as<int>();
            resolution_ = cv::Size(width, height);
            cam_rate_hz_ = node["rate_hz"].as<float>();
            auto intr_vec = vector_from_yaml<float>(node["intrinsics"]);
            intrinsic_[i] = intrinsic_t {
                .fx = intr_vec(0),
                .fy = intr_vec(1),
                .cx = intr_vec(2),
                .cy = intr_vec(3)
            };
            auto distor_vec = vector_from_yaml<double>(node["distortion_coefficients"]);
            distortion_[i] = distortion_t {
                .k1 = distor_vec(0),
                .k2 = distor_vec(1),
                .p1 = distor_vec(2),
                .p2 = distor_vec(3),
                .k3 = (distor_vec.size() > 4 ? distor_vec(4) : 0)
            };
        }
    }
    
    /// @brief load imu sensor calib and data
    void _load_motion_data() {
        if(__log_d__) std::cout << "[euroc] loading imu motion data" << std::endl;

        fs::path imu_csv = base_dir_ / "imu0" / "data.csv";
        auto csv = csvReader(imu_csv.string());
        
        size_t cnt = 0;
        while(!csv.isEnd()) {
            auto ts = csv.next<uint64_t>()[0];
            auto data = get_(ts, true);
            auto val = csv.next<double>(6);
            data->gyroscope = motion_t {val[0], val[1], val[2]};
            data->accelerometer = motion_t {val[3], val[4], val[5]};
            data->set(dataclip_t::GYRO);
            data->set(dataclip_t::ACC);
            ++cnt;
        }

        if(__log_d__) std::cout << "[euroc] loaded " << cnt << " imu motion data" << std::endl;


        // read yaml
        fs::path imu_yaml = base_dir_ / "imu0" / "sensor.yaml";
        YAML::Node node = YAML::LoadFile(imu_yaml.string());
        imu_rate_hz_ = node["rate_hz"].as<float>();
    }

    /// @brief read image data
    cv::Mat _read_image(const uint64_t ns, const int cam_id) {
        fs::path cam = base_dir_ / ("cam" + std::to_string(cam_id)) / "data" / (std::to_string(ns) + ".png");
        if(!fs::exists(cam)) {
            throw std::runtime_error("file not found: " + cam.string());
        }
        cv::Mat img = cv::imread(cam.string(), cv::IMREAD_UNCHANGED);
        return undistort_image(img, intrinsic_[cam_id], distortion_[cam_id]);
    }

    /// @brief preload all images
    void _pre_load() {
        if(__log_d__) std::cout << "[euroc] preloading all images" << std::endl;
        
        size_t cnt = 0;
        for(const auto& d : data_) {
            if(d->available[dataclip_t::CAM0] && d->cam0.empty()) {
                d->cam0 = _read_image(d->timestamp.ns, 0);
                ++cnt;
            }
            if(stereo_enabled_ && d->available[dataclip_t::CAM1] && d->cam1.empty()) {
                d->cam1 = _read_image(d->timestamp.ns, 1);
                ++cnt;
            }
        }

        if(__log_d__) std::cout << "[euroc] " << cnt << " images loaded" << std::endl;

    }

    cv::Size resolution_;
    intrinsic_t intrinsic_[2];
    distortion_t distortion_[2];
    float cam_rate_hz_;
    float imu_rate_hz_;
    // Eigen::Matrix4f T_BS_[3];

    const bool motion_enabled_;
    const bool stereo_enabled_;
    const bool auto_free_enabled_;
}; // class EuRoC



/**
 * @brief ADVIO dataset
 * @note [Get] Sequentially get data using next() function
 * @note [Device] For iPhone captures only
*/
class ADVIO : public Base {
/// @brief intrinsic parameters for iPhone
const intrinsic_t seq_intrinsics_[4] = {
    intrinsic_t {  // seq 1-12
        .fx = 1077.2, .fy = 1079.3, .cx = 362.14, .cy = 636.39
    },
    intrinsic_t {  // seq 13-17
        .fx = 1082.4, .fy = 1084.4, .cx = 364.68, .cy = 643.31
    },
    intrinsic_t {  // seq 18-19
        .fx = 1076.9, .fy = 1078.5, .cx = 360.96, .cy = 639.31
    },
    intrinsic_t {  // seq 20-23
        .fx = 1081.1, .fy = 1082.1, .cx = 359.59, .cy = 640.79
    }
};

/// @brief distortion parameters for iPhone
const distortion_t seq_distortions_[4] = {
    distortion_t {  // seq 1-12
        .k1 = -0.0003, .k2 = -0.0009, .p1 = 0.0478, .p2 = 0.0339
    },
    distortion_t {  // seq 13-17
        .k1 = 0.0007, .k2 = -0.0002, .p1 = 0.0366, .p2 = 0.0803
    },
    distortion_t {  // seq 18-19
        .k1 = -0.0054, .k2 = 0.0473, .p1 = 0.0510, .p2 = -0.0354
    },
    distortion_t {  // seq 20-23
        .k1 = 0.0009, .k2 = -0.0018, .p1 = 0.0556, .p2 = -0.0454
    }
};

/// @brief get group by seq id
static int get_seq_group(const int seq_id) {
    switch (seq_id) {
        case 1 ... 12: return 0;
        case 13 ... 17: return 1;
        case 18 ... 19: return 2;
        case 20 ... 23: return 3;
        default: return -1;
    }
}

public:
    /**
     * @brief Constructor
     * @param advio_dir advio root directory
     * @param seq_id sequence id
     * @param motion_d motion data enabled (optional, true)
     * @param auto_free auto free previous data (optional, false)
     * @param pre_load pre load all images (optional, false)
     * @note The ADVIO dataset is stored in a directory with the following structure:
    */
    ADVIO(const std::string& advio_dir, const int seq_id, const bool motion_d = true, const bool auto_free = false, const bool pre_load = false)
        : Base(advio_dir), motion_enabled_(motion_d), auto_free_enabled_(auto_free)
    {
        seq_dir_ = base_dir_  
                / ("advio-" + std::string(seq_id < 10 ? "0" : "") + std::to_string(seq_id))
                / "iphone";
        
        if(__log_d__) {
            std::cout << "[dataset] load advio dataset from " << seq_dir_.string() << std::endl;
            std::cout << "[advio] imu motion " << (motion_d ? "enabled" : "disabled") << std::endl;
        }   

        const int group_id = get_seq_group(seq_id);
        if(group_id < 0) {
            throw std::runtime_error("invalid seq id: " + std::to_string(seq_id));
        }

        // parameters
        intrinsic_ = seq_intrinsics_[group_id];
        distortion_ = seq_distortions_[group_id];

        // load data
        _load_frame_data(pre_load);
        if(motion_d) _load_motion_data();

        _order_by_time();
    }

    std::shared_ptr<dataclip_t> next() override {
        if(data_pt_ >= data_.size()) return nullptr;
        auto data_at_pt = data_[data_pt_];
        // read for camera data
        const uint64_t ts = data_at_pt->timestamp.ns;
        // check avaliable for cam0 and load status
        if(data_at_pt->has_cam0() && data_at_pt->cam0.empty()) {
            cap_ >> data_at_pt->cam0;
            // flip
            cv::flip(data_at_pt->cam0, data_at_pt->cam0, -1);
            data_at_pt->cam0 = undistort_image(data_at_pt->cam0, intrinsic_, distortion_);
        }

        if(__log_d__) std::cout << "[advio] " << (data_pt_ + 1) << " / " << data_.size() << " at " << ts << std::endl;

        if(auto_free_enabled_) _free_last();

        return data_[data_pt_++];
    }

private:
    /// @brief load frame data
    void _load_frame_data(bool preload) {
        // read mov
        fs::path frame_mov = seq_dir_ / "frames.mov";
        cap_.open(frame_mov.string());
        if(!cap_.isOpened()) {
            throw std::runtime_error("Cannot open video file: " + frame_mov.string());
        }

        resolution_ = cv::Size(cap_.get(cv::CAP_PROP_FRAME_WIDTH), cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
        fps_ = cap_.get(cv::CAP_PROP_FPS);
        auto frame_count = cap_.get(cv::CAP_PROP_FRAME_COUNT);

        if(__log_d__ && preload) {
            std::cout << "[advio] preloading all image frames" << std::endl;
        }

        fs::path frame_csv = seq_dir_ / "frames.csv";
        auto csv = csvReader(frame_csv.string());

        size_t cnt = 0;
        while(!csv.isEnd()) {
            double t = csv.next<double>()[0]; csv.ignore();
            uint64_t time_ns = timestamp_t::from_second<double>(t).ns;
            auto data = get_(time_ns, true);
            data->set(dataclip_t::CAM0);
            if(preload) {
                cap_ >> data->cam0;
                data->cam0 = undistort_image(data->cam0, intrinsic_, distortion_);
            }
            ++cnt;
        }

        if(__log_d__) std::cout << "[advio] loaded " << cnt << " frames for video" << std::endl;

    }

    /// @brief load motion data
    void _load_motion_data() {
        // for accelerometer
        fs::path acc_csv = seq_dir_ / "accelerometer.csv";
        auto csv = csvReader(acc_csv.string());
        
        size_t cnt = 0;
        while(!csv.isEnd()) {
            double t = csv.next<double>()[0];
            uint64_t time_ns = timestamp_t::from_second<double>(t).ns;
            auto data = get_(time_ns, true);
            auto acc_val = csv.next<double>(3);
            data->accelerometer = motion_t {acc_val[0], acc_val[1], acc_val[2]};
            data->set(dataclip_t::ACC);
            ++cnt;
        }

        if(__log_d__) std::cout << "[advio] loaded " << cnt << " accelerometer data" << std::endl;
    
        // for gyroscope
        fs::path gyro_csv = seq_dir_ / "gyro.csv";
        csv = csvReader(gyro_csv.string());

        cnt = 0;
        while(!csv.isEnd()) {
            double t = csv.next<double>()[0];
            uint64_t time_ns = timestamp_t::from_second<double>(t).ns;
            auto data = get_(time_ns, true);
            auto gyro_val = csv.next<double>(3);
            data->gyroscope = motion_t {gyro_val[0], gyro_val[1], gyro_val[2]};
            data->set(dataclip_t::GYRO);
            ++cnt;
        }

        if(__log_d__) std::cout << "[advio] loaded " << cnt << " gyroscope data" << std::endl;
    }

    fs::path seq_dir_;
    cv::VideoCapture cap_;
    int fps_;

    cv::Size resolution_;
    intrinsic_t intrinsic_;
    distortion_t distortion_;

    const bool motion_enabled_;
    const bool auto_free_enabled_;

}; // class ADVIO



/**
 * @brief Image sequence dataset
 * @note [Get] Sequentially get data using next() function
 * @note [Type] Only mono or stereo image data is supported.
*/
class ImageSeq : public Base {
public:
    /**
     * @brief Constructor
     * @param img_dir base image directory, structured: $base/[$cam0, $cam1, ($yaml)]
     * @param fps frame per second
     * @param stereo_d stereo data enabled (optional, false)
     * @param auto_free auto free previous data (optional, false)
     * @param yaml_path yaml file path (optional, "")
     * @note [Predict] If no yaml file is provided, the intrinsic parameters are predicted.
    */
    ImageSeq(const std::string& img_dir, const float fps, const bool stereo_d = false, const bool auto_free = false, const std::string& yaml_path = "") 
        : Base(img_dir), fps_(fps), stereo_enabled_(stereo_d), auto_free_enabled_(auto_free)
    {
        _load_camera_data();
        _load_yaml(yaml_path);
        _order_by_time();
    }

    /**
     * @brief get next dataclip
     * @note [Return] return nullptr if no next data available
    */
    std::shared_ptr<dataclip_t> next() override {
        if(data_pt_ >= data_.size()) return nullptr;
        auto data_at_pt = data_[data_pt_];
        // read for camera data
        const uint64_t ts = data_at_pt->timestamp.ns;
        // check avaliable for cam0 and load status
        if(data_at_pt->available[dataclip_t::CAM0] && data_at_pt->cam0.empty()) {
            data_at_pt->cam0 = _read_image(ts, 0);
        }
        // check avaliable for cam1 (stereo) and load status
        if(stereo_enabled_ && data_at_pt->available[dataclip_t::CAM1] && data_at_pt->cam1.empty()) {
            data_at_pt->cam1 = _read_image(ts, 1);
        }

        if(__log_d__) std::cout << "[images] " << (data_pt_ + 1) << " / " << data_.size() << " at " << ts << std::endl;

        if(auto_free_enabled_) _free_last();

        return data_[data_pt_++];
    }

private:
    /// @brief load camera data
    void _load_camera_data() {
        if(__log_d__) std::cout << "[images] loading camera data" << std::endl;

        for(int i = 0; i <= static_cast<int>(stereo_enabled_); i++){
            fs::path cam_data = base_dir_ / ("cam" + std::to_string(i)) ;
            fs::path cam_yaml = base_dir_ / "sensor.yaml";
            
            size_t timestamp = 1;
            const size_t ts_step = 1e9 / fps_;
            for(const auto& entry : fs::directory_iterator(cam_data)) {
                if(entry.is_directory()) continue;

                const auto& path = entry.path();
                const auto ext = path.extension().string();
                if(ext != ".png" && ext != ".jpg") continue;
                
                image_files_[i].insert({timestamp, path});
                auto data = get_(timestamp, true);
                data->available[i] = true;
                
                timestamp += ts_step;
            }
        }

        if(__log_d__) std::cout << "[images] " << image_files_[0].size() << " frames loaded" << std::endl;
    
    }
    
    /// @brief load yaml data
    void _load_yaml(const std::string yaml_file) {
        if(yaml_file.empty()) {
            if(__log_d__) std::cout << "[images] no specific yaml file, predict intrinsic" << std::endl; 
            const auto path = image_files_[0].begin()->second;
            cv::Mat img = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
            resolution_ = img.size();

            for(int i = 0; i < static_cast<int>(stereo_enabled_); i++) {
                intrinsic_[i] = predict_intrinsic(resolution_.width, resolution_.height);
                distortion_[i] = distortion_t {0, 0, 0, 0, 0};
            }
            return;
        }

        // read yaml
        YAML::Node node = YAML::LoadFile(yaml_file);
        int width = node["resolution"][0].as<int>();
        int height = node["resolution"][1].as<int>();
        resolution_ = cv::Size(width, height);
        for(int i = 0; i < static_cast<int>(stereo_enabled_); i++) {
            const auto cam_node = node["cam" + std::to_string(i)];
            if(cam_node.IsNull() || cam_node["intrinsics"].IsNull() || cam_node["distortion_coefficients"].IsNull()) {
                throw std::runtime_error("[images] invalid yaml file for cam" + std::to_string(i));
            } 
            // for intrinsic 
            auto intr_vec = vector_from_yaml<float>(cam_node["intrinsics"]);
            intrinsic_[i] = intrinsic_t {
                .fx = intr_vec(0),
                .fy = intr_vec(1),
                .cx = intr_vec(2),
                .cy = intr_vec(3)
            };
            // for distortion
            auto distor_vec = vector_from_yaml<double>(cam_node["distortion_coefficients"]);
            distortion_[i] = distortion_t {
                .k1 = distor_vec(0),
                .k2 = distor_vec(1),
                .p1 = distor_vec(2),
                .p2 = distor_vec(3),
                .k3 = (distor_vec.size() > 4 ? distor_vec(4) : 0)
            };
        }
    }

    /// @brief read image data
    cv::Mat _read_image(const uint64_t ns, const int cam_id = 0) {
        const fs::path cam = image_files_[cam_id][ns];
        if(!fs::exists(cam)) {
            throw std::runtime_error("file not found: " + cam.string());
        }
        cv::Mat img = cv::imread(cam.string(), cv::IMREAD_UNCHANGED);
        return undistort_image(img, intrinsic_[cam_id], distortion_[cam_id]);
    }

    /// @brief preload all images
    void _pre_load() {
        if(__log_d__) std::cout << "[images] preloading all images" << std::endl;
        
        size_t cnt = 0;
        for(const auto& d : data_) {
            if(d->available[dataclip_t::CAM0] && d->cam0.empty()) {
                d->cam0 = _read_image(d->timestamp.ns, 0);
                ++cnt;
            }
            if(stereo_enabled_ && d->available[dataclip_t::CAM1] && d->cam1.empty()) {
                d->cam1 = _read_image(d->timestamp.ns, 1);
                ++cnt;
            }
        }

        if(__log_d__) std::cout << "[images] " << cnt << " images loaded" << std::endl;
    }

    std::unordered_map<uint64_t, fs::path> image_files_[2];

    cv::Size resolution_;
    intrinsic_t intrinsic_[2];
    distortion_t distortion_[2];

    const float fps_;
    const bool stereo_enabled_;
    const bool auto_free_enabled_;

}; // class Image


class VideoSeq : public Base {
public:
    /**
     * @brief Constructor
     * @param video_file video file path
     * @param fps frame per second (optional, 0)
     * @param yaml_path yaml file path (optional, "")
     * @param auto_free auto free previous data (optional, false)
     * @param preload pre load all images (optional, false)
     * @note [Predict] If no yaml file is provided, the intrinsic parameters are predicted.
     * @note [Predict] If no fps is provided, the video file fps is used.
    */
    VideoSeq(const std::string& video_file, const float fps = 0, const std::string& yaml_path = "", const bool auto_free = false, const bool preload = false) 
        : Base(video_file), fps_(fps), auto_free_enabled_(auto_free)
    {
        if(__log_d__) {
            std::cout << "[dataset] load video dataset from " << video_file << std::endl;
            std::cout << "[video] fps " << fps << std::endl;
        }
        _load_video(preload);
        _load_yaml(yaml_path);
    }

    /**
     * @brief get next dataclip
     * @note [Return] return nullptr if no frames left in video
    */
    std::shared_ptr<dataclip_t> next() override {
        if(data_pt_ >= data_.size()) return nullptr;
        auto data_at_pt = data_[data_pt_];
        // read for camera data
        if(data_at_pt->available[dataclip_t::CAM0] && data_at_pt->cam0.empty()) {
            // cap_.set(cv::CAP_PROP_POS_FRAMES, data_pt_);
            cap_ >> data_at_pt->cam0;
            data_at_pt->cam0 = undistort_image(data_at_pt->cam0, intrinsic_, distortion_);
        }

        if(__log_d__) std::cout << "[video] " << (data_pt_ + 1) << " / " << data_.size() << " at " << ts_ << std::endl;

        if(auto_free_enabled_) _free_last();
        
        return data_[data_pt_++];
    }

private:
    /// @brief load video data
    void _load_video(bool preload) {
        if(__log_d__) std::cout << "[video] loading video data" << std::endl;

        cap_.open(base_dir_.string());
        if(!cap_.isOpened()) {
            throw std::runtime_error("Cannot open video file: " + base_dir_.string());
        }

        resolution_ = cv::Size(cap_.get(cv::CAP_PROP_FRAME_WIDTH), cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
        if(fps_ == 0) fps_ = cap_.get(cv::CAP_PROP_FPS);
        auto frame_count = cap_.get(cv::CAP_PROP_FRAME_COUNT);

        if(__log_d__ && preload) {
            std::cout << "[video] preloading all image frames" << std::endl;
        }

        uint64_t ts = 1, ts_step = 1e9 / fps_;
        for(size_t i = 0; i < frame_count; i++, ts += ts_step) {
            auto data = get_(ts, true);
            data->available[dataclip_t::CAM0] = true;
            if(preload) {
                cap_ >> data->cam0;
                data->cam0 = undistort_image(data->cam0, intrinsic_, distortion_);
            }
        }
    }

    /// @brief load yaml data
    void _load_yaml(const std::string& yaml_file) {
        if(yaml_file.empty()) {
            if(__log_d__) std::cout << "[video] no specific yaml file, predict intrinsic" << std::endl; 
            intrinsic_ = predict_intrinsic(resolution_.width, resolution_.height);
            distortion_ = distortion_t {0, 0, 0, 0, 0};
            return;
        }

        // read yaml
        YAML::Node cam_node = YAML::LoadFile(yaml_file);
        if(cam_node.IsNull() || cam_node["intrinsics"].IsNull() || cam_node["distortion_coefficients"].IsNull()) {
            throw std::runtime_error("[video] invalid yaml file for video");
        } 
        // for intrinsic 
        auto intr_vec = vector_from_yaml<float>(cam_node["intrinsics"]);
        intrinsic_ = intrinsic_t {
            .fx = intr_vec(0),
            .fy = intr_vec(1),
            .cx = intr_vec(2),
            .cy = intr_vec(3)
        };
        // for distortion
        auto distor_vec = vector_from_yaml<double>(cam_node["distortion_coefficients"]);
        distortion_ = distortion_t {
            .k1 = distor_vec(0),
            .k2 = distor_vec(1),
            .p1 = distor_vec(2),
            .p2 = distor_vec(3),
            .k3 = (distor_vec.size() > 4 ? distor_vec(4) : 0)
        };
    }



    uint64_t ts_ = 1;
    cv::VideoCapture cap_;

    cv::Size resolution_;
    intrinsic_t intrinsic_;
    distortion_t distortion_;

    float fps_;
    const bool auto_free_enabled_;

}; // class Video


} // namespace dataset