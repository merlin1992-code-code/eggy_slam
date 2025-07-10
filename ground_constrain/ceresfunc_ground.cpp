/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-07-10 10:14:35
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-10 10:14:36
 */
#include "ceresfunc_ground.h"

Eigen::Vector3d sqrt_information_vec(0.0000205, 0.0000205, 0.0001);
Eigen::Matrix<double, 3, 3> Cost_NavState_PR_Ground::sqrt_information = sqrt_information_vec.asDiagonal().inverse();
Eigen::VectorXf Cost_NavState_PR_Ground::init_ground_plane_coeff(4);