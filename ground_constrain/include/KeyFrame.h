/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-07-09 19:07:00
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-10 10:11:12
 */
#ifndef KEY_FRAME_H
#define KEY_FRAME_H

#pragma once
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct KeyFrame {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix4d pose; // 世界到雷达的变换
    Eigen::VectorXf ground_plane_coeff; // 4维，ax+by+cz+d=0
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr filtered_cloud;

    KeyFrame(): filtered_cloud(new pcl::PointCloud<pcl::PointXYZINormal>()) {}
};
#endif // KEY_FRAME_H

