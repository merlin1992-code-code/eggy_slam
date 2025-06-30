/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-06-24 09:41:01
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-06-27 16:07:44
 */
#include "map_builder.h"
MapBuilder::MapBuilder(Config &config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf)
{
    m_imu_processor = std::make_shared<IMUProcessor>(config, kf);
    m_lidar_processor = std::make_shared<LidarProcessor>(config, kf);
    m_status = BuilderStatus::IMU_INIT;
}

void MapBuilder::process(SyncPackage &package)
{
    if (m_status == BuilderStatus::IMU_INIT)
    {
        if (m_imu_processor->initialize(package))
            m_status = BuilderStatus::MAP_INIT;
        return;
    }
    // std::cout<<"ooooooooo"<<std::endl;
    // printCloudRange(package.cloud, "package.cloud", BuilderStatusToString(m_status));

    m_imu_processor->undistort(package);
    // printCloudRange(package.cloud, "package.cloud");

    if (m_status == BuilderStatus::MAP_INIT)
    {   
        printCloudRange(package.cloud, "package.cloud", BuilderStatusToString(m_status));
        CloudType::Ptr cloud_world = LidarProcessor::transformCloud(package.cloud, m_lidar_processor->r_wl(), m_lidar_processor->t_wl());
        printCloudRange(cloud_world, "cloud_world", BuilderStatusToString(m_status));
        m_lidar_processor->initCloudMap(cloud_world->points);
        m_status = BuilderStatus::MAPPING;
        return;
    }
    printCloudRange(package.cloud, "package.cloud", BuilderStatusToString(m_status));
    
    m_lidar_processor->process(package);
}