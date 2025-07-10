/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-07-10 10:11:37
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-10 21:17:25
 */

#ifndef EXTRACT_FIT_GROUND_PLANE_H
#define EXTRACT_FIT_GROUND_PLANE_H

#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

template <typename PointT>
void extractGroundPoints(
    const pcl::PointCloud<PointT> &in_cloud,
    pcl::PointCloud<PointT> &ground_cloud,
    float normal_y_threshold = 0.08f)
{
    ground_cloud.clear();
    for (const auto &p : in_cloud.points)
    {
        if (std::fabs(p.normal_y + 1.0f) < normal_y_threshold)
        {
            ground_cloud.push_back(p);
        }
    }
}
#endif // EXTRACT_FIT_GROUND_PLANE_H