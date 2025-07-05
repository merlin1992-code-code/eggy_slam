#ifndef TRAJECTORY_H
#define TRAJECTORY_H
/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-07-05 15:51:19
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-05 15:51:37
 */
#pragma once
#include <vector>
#include <string>
#include <Eigen/Dense>

struct TrajectoryPose {
    double time;
    Eigen::Vector3d pos;
    Eigen::Matrix3d rot;
};

class Trajectory {
public:
    std::vector<TrajectoryPose> poses;

    void add(double time, const Eigen::Vector3d& pos, const Eigen::Matrix3d& rot) {
        poses.push_back({time, pos, rot});
    }

    bool saveToCSV(const std::string& filename) const;
    bool loadFromCSV(const std::string& filename);
};
#endif // TRAJECTORY_H