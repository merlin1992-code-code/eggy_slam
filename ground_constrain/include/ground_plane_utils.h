/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-07-10 10:13:22
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-13 22:27:53
 */
#ifndef GROUND_PLANE_UTILS_H
#define GROUND_PLANE_UTILS_H
#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

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
Eigen::Vector4f get_plane_coeffs_pca(const pcl::PointCloud<PointT> &cloud)
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

// RANSAC 拟合地面平面，输出 ax+by+cz+d=0 格式的coeff
template <typename PointT>
Eigen::Vector4f get_plane_coeffs(const pcl::PointCloud<PointT> &cloud,
                                 double ransac_dist_thresh = 0.05)
{
    Eigen::Vector4f coeff;
    if (cloud.size() < 10)
    {
        coeff << 0, 1, 0, 0; // 点太少，默认y向上（可按需求改z向上）
        return coeff;
    }

    typename pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>(cloud));
    typename pcl::SampleConsensusModelPlane<PointT>::Ptr model(
        new pcl::SampleConsensusModelPlane<PointT>(cloud_ptr));
    pcl::RandomSampleConsensus<PointT> ransac(model);
    ransac.setDistanceThreshold(ransac_dist_thresh);

    if (!ransac.computeModel())
    {
        coeff << 0, 1, 0, 0; // RANSAC失败时兜底
        return coeff;
    }

    Eigen::VectorXf model_coeffs;
    ransac.getModelCoefficients(model_coeffs);
    if (model_coeffs.size() != 4)
    {
        coeff << 0, 1, 0, 0;
        return coeff;
    }
    coeff << model_coeffs[0], model_coeffs[1], model_coeffs[2], model_coeffs[3];
    // 法向量归一化
    float norm = coeff.head<3>().norm();
    if (norm > 1e-6f)
        coeff /= norm;

    // 保证法线朝上（z轴为上。按你实际坐标系调整为y轴也可）
    if (coeff.head<3>().dot(Eigen::Vector3f::UnitZ()) < 0.0f)
        coeff *= -1.0f;

    return coeff;
}

// 根据拟合平面系数，将距离平面小于阈值的点筛选出来
template <typename PointT>
void ground_filtered(const pcl::PointCloud<PointT> &input_cloud,
                     const Eigen::Vector4f &plane_coeff,
                     pcl::PointCloud<PointT> &output_cloud,
                     float dist_thresh = 0.08)
{
    output_cloud.clear();
    float norm = plane_coeff.head<3>().norm();
    if (norm < 1e-6f)
        return; // 避免除0

    for (const auto &p : input_cloud.points)
    {
        float dist = std::abs(plane_coeff[0] * p.x + plane_coeff[1] * p.y + plane_coeff[2] * p.z + plane_coeff[3]) / norm;
        if (dist < dist_thresh)
        {
            output_cloud.push_back(p);
        }
    }
}

#endif // GROUND_PLANE_UTILS_H