/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-07-10 10:12:03
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-10 12:06:45
 */
#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

#include "KeyFrame.h"
#include "extract_fit_ground_plane.h"
#include "ground_plane_utils.h"

// 输入：filtered_pcd, pose (Eigen::Matrix4d)
KeyFrame createKeyFrame(const pcl::PointCloud<pcl::PointXYZINormal> &filtered_pcd, const Eigen::Matrix4d &pose)
{
    KeyFrame kf;
    *kf.filtered_cloud = filtered_pcd;
    kf.pose = pose;

    // 地面点提取
    pcl::PointCloud<pcl::PointXYZINormal> ground_cloud;
    extractGroundPoints(filtered_pcd, ground_cloud);

    // 地面平面拟合（调用 Livox-Mapping 一致的实现）
    Eigen::Vector4f plane_coeff = get_plane_coeffs(ground_cloud);
    kf.ground_plane_coeff = plane_coeff;

    return kf;
}
