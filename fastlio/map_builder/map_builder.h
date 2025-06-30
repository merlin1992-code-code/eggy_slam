/*
 * @Description: Do not Edit

 * @Author: hao.lin (voyah perception)
 * @Date: 2025-06-24 09:41:01
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-06-25 10:59:41
 */
#pragma once
#include "imu_processor.h"
#include "lidar_processor.h"

enum BuilderStatus
{
    IMU_INIT,
    MAP_INIT,
    MAPPING
};

inline const char* BuilderStatusToString(BuilderStatus status) {
    switch (status) {
        case IMU_INIT:  return "IMU_INIT";
        case MAP_INIT:  return "MAP_INIT";
        case MAPPING:   return "MAPPING";
        default:        return "OTHER";
    }
}

class MapBuilder
{
public:
    MapBuilder(Config &config, std::shared_ptr<IESKF> kf);

    void process(SyncPackage &package);
    BuilderStatus status() { return m_status; }    
    std::shared_ptr<LidarProcessor> lidar_processor(){return m_lidar_processor;}

private:
    Config m_config;
    BuilderStatus m_status;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<IMUProcessor> m_imu_processor;
    std::shared_ptr<LidarProcessor> m_lidar_processor;
};

