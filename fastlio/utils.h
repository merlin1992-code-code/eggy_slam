/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-06-24 09:43:56
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-06-24 16:20:36
 */
#pragma once
#include <iomanip>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Imu.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#define RESET "\033[0m"
#define BLACK "\033[30m"  /* Black */
#define RED "\033[31m"    /* Red */
#define GREEN "\033[32m"  /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m"   /* Blue */
#define PURPLE "\033[35m" /* Purple */
#define CYAN "\033[36m"   /* Cyan */
#define WHITE "\033[37m"  /* White */

class Utils
{
public:
    static double getSec(std_msgs::Header &header);
    static pcl::PointCloud<pcl::PointXYZINormal>::Ptr convertToPCL(const sensor_msgs::PointCloud2Ptr pc_msg, int filter_num, double min_range = 0.4, double max_range = 150.0);
    static void ParseRosbagToVectors(
        const std::string &bag_path,
        const std::string &lidar_topic,
        const std::string &imu_topic,
        std::vector<sensor_msgs::PointCloud2Ptr> &lidar_vec,
        std::vector<sensor_msgs::ImuPtr> &imu_vec);
};