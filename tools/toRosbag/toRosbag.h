/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-07-01 09:49:20
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-07 21:13:59
 */
#ifndef TO_ROSBAG_H
#define TO_ROSBAG_H

#include <pcl/io/pcd_io.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace rs128
{
    struct Point
    {
        float x;
        float y;
        float z;
        float intensity;
        uint16_t ring;
        double timestamp;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace rs128

POINT_CLOUD_REGISTER_POINT_STRUCT(
    rs128::Point,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        uint16_t, ring, ring)(double, timestamp, timestamp))

// 模板声明
template <typename PointT>
void pointCloudToMsg(const pcl::PointCloud<PointT> &cloud,
                     sensor_msgs::PointCloud2 &msg);

template <>
void pointCloudToMsg<rs128::Point>(const pcl::PointCloud<rs128::Point> &cloud,
                                   sensor_msgs::PointCloud2 &msg);

template <typename PointT>
void writePcdToBag(const std::string &pcd_folder, rosbag::Bag &bag,
                   const std::string &point_topic);

void writeImuToBag(const std::string &imu_txt, rosbag::Bag &bag,
                   const std::string &imu_topic);


std::vector<std::string> get_sorted_pcd_files(const std::string& pcd_folder);
std::vector<std::string> get_imu_lines(const std::string& imu_txt);

template <typename PointT>
void writeBatchBags(const std::string &pcd_folder, const std::string &imu_txt,
                    const std::string &bag_prefix,
                    const std::string &point_topic, const std::string &imu_topic,
                    size_t batch_size);

#endif // TO_ROSBAG_H