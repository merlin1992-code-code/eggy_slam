/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-06-24 09:41:01
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-08 11:28:32
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
    m_imu_processor->undistort(package);
    if (m_status == BuilderStatus::MAP_INIT)
    {   
        CloudType::Ptr cloud_world = LidarProcessor::transformCloud(package.cloud, m_lidar_processor->r_wl(), m_lidar_processor->t_wl());
        m_lidar_processor->initCloudMap(cloud_world->points);
        m_status = BuilderStatus::MAPPING;
        return;
    }    
    m_lidar_processor->process(package);
}