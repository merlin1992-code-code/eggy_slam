/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-06-24 09:41:01
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-06-26 15:36:22
 */
#include "commons.h"
bool esti_plane(PointVec &points, const double &thresh, V4D &out)
{
    Eigen::MatrixXd A(points.size(), 3);
    Eigen::MatrixXd b(points.size(), 1);
    A.setZero();
    b.setOnes();
    b *= -1.0;
    for (size_t i = 0; i < points.size(); i++)
    {
        A(i, 0) = points[i].x;
        A(i, 1) = points[i].y;
        A(i, 2) = points[i].z;
    }
    V3D normvec = A.colPivHouseholderQr().solve(b);
    double norm = normvec.norm();
    out[0] = normvec(0) / norm;
    out[1] = normvec(1) / norm;
    out[2] = normvec(2) / norm;
    out[3] = 1.0 / norm;
    for (size_t j = 0; j < points.size(); j++)
    {
        if (std::fabs(out(0) * points[j].x + out(1) * points[j].y + out(2) * points[j].z + out(3)) > thresh)
        {
            return false;
        }
    }
    return true;
}

float sq_dist(const PointType &p1, const PointType &p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
}

void printCloudRange(const CloudType::Ptr &cloud, const std::string &name, const std::string status)
{
    std::cout << "[范围检查] " << name << " status: " << status << std::endl;
    if (!cloud || cloud->empty())
    {
        std::cout << "[范围检查] " << name << " is empty!" << std::endl;
        return;
    }
    float min_x = cloud->points[0].x, max_x = cloud->points[0].x;
    float min_y = cloud->points[0].y, max_y = cloud->points[0].y;
    float min_z = cloud->points[0].z, max_z = cloud->points[0].z;
    for (const auto &pt : cloud->points)
    {
        if (pt.x < min_x)
            min_x = pt.x;
        if (pt.x > max_x)
            max_x = pt.x;
        if (pt.y < min_y)
            min_y = pt.y;
        if (pt.y > max_y)
            max_y = pt.y;
        if (pt.z < min_z)
            min_z = pt.z;
        if (pt.z > max_z)
            max_z = pt.z;
    }
    std::cout << "[范围检查] " << name << " x:[" << min_x << "," << max_x << "]"
              << " y:[" << min_y << "," << max_y << "]"
              << " z:[" << min_z << "," << max_z << "]" << std::endl;
}

void printCloudValid(const CloudType::Ptr &cloud)
{
    std::cout << "[NAN 无效检查] " << std::endl;
    int nan_count = 0;
    for (const auto &pt : cloud->points)
    {
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z) ||
            !std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
        {
            nan_count++;
        }
    }
    if (nan_count > 0)
    {
        std::cerr << "[警告] cloud 中有 " << nan_count << " 个无效点（NaN/Inf）!" << std::endl;
    }
    else
    {
        std::cout << "[检查通过] cloud 中没有无效点!" << std::endl;
    }
}



void checkIMUVec(const std::vector<IMUData>& imus, const std::string& tag)
{
    std::cout << "[" << tag << "] size = " << imus.size();
    if (!imus.empty()) {
        std::cout << ", first time = " << imus.front().time
                  << ", last time = " << imus.back().time << std::endl;
    } else {
        std::cout << " (empty)" << std::endl;
        return;
    }

    bool time_order_ok = true;
    bool has_nan = false;
    for (size_t i = 0; i < imus.size(); ++i) {
        const auto& imu = imus[i];
        if (i > 0 && imu.time < imus[i-1].time) {
            std::cerr << "[" << tag << "] Warning: imu[" << i << "] time < imu[" << i-1 << "] time!" << std::endl;
            time_order_ok = false;
        }
        if (!std::isfinite(imu.time) || !imu.acc.allFinite() || !imu.gyro.allFinite()) {
            std::cerr << "[" << tag << "] Warning: imu[" << i << "] has NaN/Inf!" << std::endl;
            has_nan = true;
        }
        if (i < 3) {
            std::cout << "  imu[" << i << "]: time = " << imu.time
                      << ", acc = [" << imu.acc.transpose() << "]"
                      << ", gyro = [" << imu.gyro.transpose() << "]" << std::endl;
        }
    }
    if (time_order_ok)
        std::cout << "[" << tag << "] time order OK." << std::endl;
    if (!has_nan)
        std::cout << "[" << tag << "] No NaN/Inf in IMU data." << std::endl;
}