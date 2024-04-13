#include <chrono>

#include <rdvio/rdvio.hpp>
#include "dataset.hpp"
#include "pviz.hpp"

int main(int argc, char** argv) {
    if(argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <calib_file> <config_file> <advio_root_dir> <seq_id>" << std::endl;
        return 1;
    }

    const std::string calib_file = argv[1];
    const std::string config_file = argv[2];
    const std::string root_dir = argv[3];
    const int seq_id = std::stoi(argv[4]);

    auto advio = dataset::ADVIO(root_dir, seq_id);
    dataset::set_logger(false);

    auto vio = rdvio::Odometry(calib_file, config_file);

    auto viewer_setting = pviz::Settings();
    viewer_setting.follow = true;
    auto viewer = pviz::Viewer("advio", viewer_setting);

    std::thread viewer_thread(&pviz::Viewer::run, &viewer);
    std::shared_ptr<dataset::dataclip_t> data;

    while((data = advio.next())) {
        if(data->has_gyro()) {
            auto start = std::chrono::high_resolution_clock::now();
            vio.addGyro(data->timestamp.sec(), data->gyro());

#ifndef USE_MULTI_THREADING
            auto end = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            if(elapsed > 0)
                std::cout << "[rdvio] motion tracking takes " << elapsed << "ms" << std::endl;
#else
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / 100));
#endif
        }

        if(data->has_acc()) {
            auto start = std::chrono::high_resolution_clock::now();
            vio.addAcc(data->timestamp.sec(), data->acc());

#ifndef USE_MULTI_THREADING
            auto end = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            if(elapsed > 0)
                std::cout << "[rdvio] motion tracking takes " << elapsed << "ms" << std::endl;
#else
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / 100));
#endif
        }
        
        if(data->has_cam0()) {
            auto start = std::chrono::high_resolution_clock::now();
            vio.addFrame(data->timestamp.sec(), data->cam0);
            viewer.publish_topic("input", data->cam0);

#ifndef USE_MULTI_THREADING
            auto end = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            if(elapsed > 0)
                std::cout << "[rdvio] frame tracking takes " << elapsed << "ms" << std::endl;
#else
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / 60));
#endif
        }

        if(vio.state() == 1) {
            viewer.publish_trajectory(vio.transform_world_cam());
            viewer.publish_local_point_cloud(vio.local_map());
        }

    }

    cv::waitKey(0);

    viewer.exit();
    viewer_thread.join();

    return 0;
}