/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-06-28 20:43:56
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-01 15:22:15
 */
#include <iostream>
#include <string>
#include "fastlio/lio_node.h"
#include "m-detector/dyn_node.h"

using namespace std;

int main(int argc, char **argv)
{
    std::string lio_config, dyn_config;
    for (int i = 1; i < argc - 1; ++i)
    {
        if (std::string(argv[i]) == "--lio")
            lio_config = argv[++i];
        else if (std::string(argv[i]) == "--dyn")
            dyn_config = argv[++i];
    }
    if (lio_config.empty() || dyn_config.empty())
    {
        std::cerr << "Usage: " << argv[0] << " --lio lio_config.yaml --dyn dyn_config.yaml" << std::endl;
        return 1;
    }

    auto lio_node = std::make_shared<LIONode>(lio_config);
    // lio excute outside of main thread
    // lio_node->execute();
#if 1
    auto dyn_node = std::make_shared<DynNode>(dyn_config);
    // dyn_node->execute();

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
    {
        // std::cout << "Processing frame: " << frame << std::endl;
        lio_node->lidarCB(lidar_msg);
        while (lio_node->syncPackage())
        {
            auto t1 = std::chrono::high_resolution_clock::now();
            lio_node->processPackage();
            auto t2 = std::chrono::high_resolution_clock::now();
            if (1)
            {
                auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
                std::cout << "Processing time: " << std::fixed << std::setprecision(2) << time_used << " ms" << std::endl;
            }
            if (lio_node->checkMappingStatus())
                break;

            CloudType::Ptr body_cloud = lio_node->getBodyCloud();
            CloudType::Ptr world_cloud = lio_node->getWorldCloud();
            M3D r_il = lio_node->getRIL();
            V3D t_il = lio_node->getTIL();
            dyn_node->execute_odom(body_cloud, r_il, t_il, lio_node->scan_end_time());
        }
    }
#endif 
    //

    return 0;
}