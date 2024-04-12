#pragma once

#include <memory>
#include <unordered_map>
#include <vector>
#include <atomic>
#include <mutex>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>

namespace pviz{

class Color {
public:
    // from rgba
    Color(const uint8_t r_, const uint8_t g_, const uint8_t b_, const uint8_t a_ = 255) {
        r = r_; g = g_; b = b_; a = a_;
        float_ = {
            r : r_ / 255.0f,
            g : g_ / 255.0f,
            b : b_ / 255.0f,
            a : a_ / 255.0f
        };
    }

    static Color RGBA(const uint8_t r, const uint8_t g, const uint8_t b, const uint8_t a) {
        return Color(r, g, b, a);
    }

    static Color RGBA(const uint8_t r, const uint8_t g, const uint8_t b, const float a) {
        return Color(r, g, b, a * 255);
    }

    static Color RGB(const uint8_t r, const uint8_t g, const uint8_t b) {
        return Color(r, g, b, 255);
    }

    static Color RGBA_HEX(const int32_t hex) {
        return Color(
            (hex >> 24) & 0xFF,
            (hex >> 16) & 0xFF,
            (hex >> 8) & 0xFF,
            hex & 0xFF
        );
    }

    static Color RGB_HEX(const int32_t hex, const uint8_t a = 255) {
        return Color(
            (hex >> 16) & 0xFF,
            (hex >> 8) & 0xFF,
            hex & 0xFF,
            a
        );
    }

    static Color RGB_HEX(const int32_t hex, const float a) {
        return RGB_HEX(hex, (uint8_t)(a * 255));
    }

    uint8_t r, g, b, a;
    struct {
        float r, g, b, a;
    } float_;
}; // class Color

class Settings {
public:
    ///// pangolin window
    int width = 1024;
    int height = 768;
    int focal = 500;
    float near = 0.1;
    float far = 1000.0;
    bool follow = false;
    ///// camera view
    float ex = 0.0;
    float ey = -0.7;
    float ez = -1.8;
    float lx = 0.0;
    float ly = 0.0;
    float lz = 0.0;
    float ux = 0.0;
    float uy = -1.0;
    float uz = 0.0;
    ///// bounds
    pangolin::Attach bbottom = 0.0;
    pangolin::Attach btop = 1.0;
    pangolin::Attach bleft = 0.0;
    pangolin::Attach bright = 1.0;
    ///// background
    Color bg = Color::RGB(255, 255, 255);
    Color camera = Color::RGB(0, 0, 255);
    Color kf = Color::RGB(255, 0, 0);
    Color traj = Color::RGB(0, 0, 255);
    Color lmap = Color::RGB(0, 255, 0);
    Color gmap = Color::RGB(100, 100, 100);
    //// trajectory
    float traj_linewidth = 3.0f;
    float axes_size = 0.12f;
    float lmap_size = 3.0f;
    float gmap_size = 3.0f;
    float topic_wait_ms = 10;
    

};  // class Settings

class Viewer {
public:
    // constructor
    Viewer(const std::string& title, const Settings& settings = Settings())
        : title_(title), SE(settings) 
    {}
    // destructor
    ~Viewer() {
        exit();
    }

    // main loop
    void run() {
        // Window Create 
        pangolin::CreateWindowAndBind(title_, SE.width, SE.height);
        // Issue specific OpenGl we might need
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);
        // Define Camera Render Object (for view / scene browsing)
        s_cam = pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(
                SE.width, SE.height,
                SE.focal, SE.focal,
                SE.width / 2, SE.height / 2,
                SE.near, SE.far),
            pangolin::ModelViewLookAt(
                SE.ex, SE.ey, SE.ez,
                SE.lx, SE.ly, SE.lz,
                SE.ux, SE.uy, SE.uz)
        );
        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(SE.bbottom, SE.btop, SE.bleft, SE.bright, 
                -1 * (float)SE.width / (float)SE.height)
            .SetHandler(new pangolin::Handler3D(s_cam));
        // main camear view Twc
        viewport_Twc_.SetIdentity();
        // main loop
        while(!exit_) {
            // Clear GL Buffer
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            if(SE.follow) {
                std::lock_guard<std::mutex> lock(mutex_cam_);
                s_cam.Follow(viewport_Twc_);
            }
            d_cam.Activate(s_cam);
            // back ground
            auto bg = SE.bg.float_;
            glClearColor(bg.r, bg.g, bg.b, 1.0);
            // draw
            draw_viewpose();
            draw_local_point_cloud();
            draw_keyframes();
            draw_global_point_cloud();
            draw_trajectory();
            // Finish
            pangolin::FinishFrame();
            // extra topics
            draw_topics();
        }
            
        pangolin::DestroyWindow(title_);
        cv::destroyAllWindows();

    }

    // exit loop
    void exit() {
        exit_ = true;
    }

    // trajectory (Twc)
    void publish_trajectory(const Eigen::Matrix4d& T) {
        trajectory_.push_back(T);
        Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
        Twc.block<3, 1>(0, 3) = T.block<3, 1>(0, 3);
        std::lock_guard<std::mutex> lock(mutex_cam_);
        viewport_Twc_ = pangolin::OpenGlMatrix(Twc);
    }
    void publish_trajectory(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = R;
        T.block<3, 1>(0, 3) = t;
        publish_trajectory(T);
    }

    // keyframe
    void publish_keyframe(const size_t id, const Eigen::Matrix4d& Twc) {
        keyframes_[id] = Twc;
    }
    void remove_keyframe(const size_t id) {
        if(keyframes_.count(id) > 0) keyframes_.erase(id);
    }
    void remove_keyframes() {
        keyframes_.clear();
    }

    // global map
    void publish_global_point_cloud(const size_t id, const Eigen::Vector3d& point) {
        global_point_cloud_[id] = point;
    }
    void remove_global_point_cloud(const size_t id) {
        if(global_point_cloud_.count(id) > 0) global_point_cloud_.erase(id);
    }
    void remove_global_point_clouds() {
        global_point_cloud_.clear();
    }

    // local map
    void publish_local_point_cloud(const std::vector<Eigen::Vector3d>& points, const bool copy = false) {
        if(points.size() == 0) return;
        std::lock_guard<std::mutex> lock(mutex_lmap_);
        if(copy) {
            local_point_cloud_.clear();
            local_point_cloud_.reserve(points.size());
            std::copy(points.begin(), points.end(), local_point_cloud_.begin());
        } else {
            local_point_cloud_ = std::move(points);
        }
    }
    void remove_local_point_cloud() {
        std::lock_guard<std::mutex> lock(mutex_lmap_);
        local_point_cloud_.clear();
    }

    // extra topic
    void publish_topic(const std::string& topic, const cv::Mat& img) {
        std::lock_guard<std::mutex> lock(mutex_topic_);
        topics_[topic] = img.clone();
    }

private:
    ///// draw functions
    void draw_viewpose() {
        draw_camera_axes(Eigen::Matrix4d::Identity());
        if(trajectory_.size() == 0) return;
        Eigen::Matrix4d Twi = trajectory_.back();
        draw_camera(Twi, SE.camera);
        draw_camera_axes(Twi);
    }
    void draw_camera(const Eigen::Matrix4d& Twc, const Color& c) {
        pangolin::OpenGlMatrix Twc_gl(Twc);
        draw_camera(Twc_gl, c);
    }
    void draw_camera(const pangolin::OpenGlMatrix& Twc, const Color& c) {
        const float w = 0.08; // camera size;
        const float h = w * 0.75;
        const float z = w * 0.6;

        glPushMatrix();
        glMultMatrixd(Twc.m);
        glLineWidth(3); // camera line width

        glColor3f(c.float_.r, c.float_.g, c.float_.b);
        
        glBegin(GL_LINES);
        // camera body
        glVertex3f(0,0,0); glVertex3f(w,h,z);
        glVertex3f(0,0,0); glVertex3f(w,-h,z);
        glVertex3f(0,0,0); glVertex3f(-w,-h,z);
        glVertex3f(0,0,0); glVertex3f(-w,h,z);
        // border
        glVertex3f(w,h,z); glVertex3f(w,-h,z);
        glVertex3f(-w,h,z); glVertex3f(-w,-h,z);
        glVertex3f(-w,h,z); glVertex3f(w,h,z);
        glVertex3f(-w,-h,z); glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }

    void draw_camera_axes(const pangolin::OpenGlMatrix& Twc) {
        const float w = SE.axes_size; // camera axes size;

        glPushMatrix();
        glMultMatrixd(Twc.m);
        glLineWidth(10); // camera line width

        glBegin(GL_LINES);
        // x
        glColor3f(1, 0, 0);
        glVertex3f(0,0,0); glVertex3f(w,0,0);
        // y
        glColor3f(0, 1, 0);
        glVertex3f(0,0,0); glVertex3f(0,w,0);
        // z
        glColor3f(0, 0, 1);
        glVertex3f(0,0,0); glVertex3f(0,0,w);
        glEnd();

        glPopMatrix();
    }
    void draw_camera_axes(const Eigen::Matrix4d& Twc) {
        pangolin::OpenGlMatrix Twc_gl(Twc);
        draw_camera_axes(Twc_gl);
    }

    void draw_trajectory() {
        glLineWidth(SE.traj_linewidth);
        glBegin(GL_LINE_STRIP);
        auto color = SE.traj.float_;
        for(const auto& m : trajectory_) {
            glColor3f(color.r, color.g, color.b);
            glVertex3d(m(0, 3), m(1, 3), m(2, 3));
        }
        glEnd();
    }

    void draw_keyframes() {
        for(const auto& kf : keyframes_) {
            draw_camera(kf.second, SE.kf);
        }
    }

    void draw_global_point_cloud() {
        glPointSize(SE.gmap_size);
        glBegin(GL_POINTS);
        auto color = SE.gmap.float_;
        for(const auto& p : global_point_cloud_) {
            glColor3f(color.r, color.g, color.b);
            glVertex3d(p.second(0), p.second(1), p.second(2));
        }
        glEnd();
    }

    void draw_local_point_cloud() {
        glPointSize(SE.lmap_size);
        glBegin(GL_POINTS);
        auto color = SE.lmap.float_;
        {
            std::lock_guard<std::mutex> lock(mutex_lmap_);
            for(const auto& p : local_point_cloud_) {
                glColor3f(color.r, color.g, color.b);
                glVertex3d(p(0), p(1), p(2));
            }
        }
        glEnd();
    }

    void draw_topics() {
        {
            std::lock_guard<std::mutex> lock(mutex_topic_);
            for(const auto& topic : topics_) {
                cv::Mat img = topic.second;
                if(img.empty()) continue;
                cv::imshow(topic.first, img);
            } 
        }
        cv::waitKey(SE.topic_wait_ms);
    }

    ///// trajectory
    pangolin::OpenGlMatrix viewport_Twc_;
    std::vector<Eigen::Matrix4d> trajectory_;
    ///// 3d map
    std::vector<Eigen::Vector3d> local_point_cloud_;
    std::unordered_map<size_t, Eigen::Vector3d> global_point_cloud_;
    ///// keyframe
    std::unordered_map<size_t, Eigen::Matrix4d> keyframes_;
    ///// extra topic
    std::unordered_map<std::string, cv::Mat> topics_;

    ///// pangolin settings
    const Settings SE;
    const std::string title_;   // title
    std::atomic<bool> exit_ = false;    // exit signal
    pangolin::OpenGlRenderState s_cam;

    ///// mutex
    std::mutex mutex_cam_;
    std::mutex mutex_lmap_;
    std::mutex mutex_topic_;

}; // class Viewer

} // namespace pviz
