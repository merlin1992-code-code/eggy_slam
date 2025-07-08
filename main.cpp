/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-06-28 20:43:56
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-08 12:56:28
 */
#include <iostream>
#include <string>
#include "fastlio/lio_node.h"
#include "m-detector/dyn_node.h"

using namespace std;

enum RunMode
{
    MODE_LIO,
    MODE_FUSION
};

int main(int argc, char **argv)
{
    std::string lio_config, dyn_config;
    RunMode mode = MODE_FUSION; // 默认FUSION模式
    for (int i = 1; i < argc; ++i)
    {
        if (std::string(argv[i]) == "--lio")
            lio_config = argv[++i];
        else if (std::string(argv[i]) == "--dyn")
            dyn_config = argv[++i];
        else if (std::string(argv[i]) == "--fusion")
            mode = MODE_FUSION;
        else if (std::string(argv[i]) == "--mode_lio")
            mode = MODE_LIO;
    }
    if (lio_config.empty() || dyn_config.empty())
    {
        std::cerr << "Usage: " << argv[0] << " --lio lio_config.yaml --dyn dyn_config.yaml" << std::endl;
        return 1;
    }

    auto lio_node = std::make_shared<LIONode>(lio_config);
    lio_node->init_buffer();
    auto dyn_node = std::make_shared<DynNode>(dyn_config);
    auto lio_second_node = std::make_shared<LIONodeSecond>(lio_config, dyn_node->getStdPcdDir(), lio_node->getImuVec());
    // lio excute outside of main thread
    //
    switch (mode)
    {
    case MODE_FUSION:
    {
        std::cout << "Running in Fusion mode." << std::endl;
        auto lidar_vec = lio_node->getLidarVec();
        auto imu_vec = lio_node->getImuVec();
        if (lidar_vec.empty() || imu_vec.empty())
        {
            std::cerr << "Lidar or IMU data is empty, please check the rosbag file." << std::endl;
            return 1;
        }

        BOOST_FOREACH (const sensor_msgs::ImuPtr &imu_msg, imu_vec)
        {
            lio_node->imuCB(imu_msg);
        }
        BOOST_FOREACH (const sensor_msgs::PointCloud2Ptr &lidar_msg, lidar_vec)
        //for (size_t i = 0; i < 20; ++i)
        {
            //auto lidar_msg = lidar_vec[i];
            lio_node->lidarCB(lidar_msg);
            while (lio_node->syncPackage())
            {
                auto t1 = std::chrono::high_resolution_clock::now();
                lio_node->processPackage();
                auto t2 = std::chrono::high_resolution_clock::now();
                if (1)
                {
                    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
                    std::cout << "LIO Processing time: " << std::fixed << std::setprecision(2) << time_used << " ms" << std::endl;
                }
                if (lio_node->checkMappingStatus())
                    break;

                CloudType::Ptr lidar_cloud = lio_node->getLidarCloud();

                M3D r_wl = lio_node->getRWL();
                V3D t_wl = lio_node->getTWL();
                lio_node->debug_pose();
                dyn_node->execute(lidar_cloud, r_wl, t_wl, lio_node->scan_end_time());
            }
        }
        break;
    }
    case MODE_LIO:
    {
        std::cout << "Running in LIO mode." << std::endl;
        lio_node->execute();
        break;
    }
    default:
        std::cerr << "Unknown mode, please specify --fusion, --lio or --dyn." << std::endl;
        return 1;
    }
    return 0;
}