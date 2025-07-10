/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-07-10 10:13:22
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-10 12:06:28
 */
#ifndef GROUND_PLANE_UTILS_H
#define GROUND_PLANE_UTILS_H
#pragma once
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> ominus(
    const Eigen::MatrixBase<Derived> &p1,
    const Eigen::MatrixBase<Derived> &p2)
{
    using Scalar = typename Derived::Scalar;
    Eigen::Matrix<Scalar, 3, 1> res;
    res = p1.template segment<3>(0) - p2.template segment<3>(0);
    return res;
}

template <typename PointT>
Eigen::Vector4f get_plane_coeffs(const pcl::PointCloud<PointT> &cloud)
{
    Eigen::Vector4f coeff;
    if (cloud.size() < 10)
    {
        coeff << 0, 1, 0, 0; // 点太少，默认y向上
        return coeff;
    }
    // 1. 计算质心
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(cloud, centroid);

    // 2. 计算协方差矩阵
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(cloud, centroid, covariance);

    // 3. 求协方差矩阵最小特征值的特征向量（即平面法向）
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Vector3f normal = eigen_solver.eigenvectors().col(0);

    // 4. 平面参数 ax+by+cz+d=0
    float d = -normal.dot(centroid.head<3>());
    coeff << normal, d;
    coeff.normalize(); // Livox-Mapping 源码中有 normalize
    return coeff;
}

#endif // GROUND_PLANE_UTILS_H