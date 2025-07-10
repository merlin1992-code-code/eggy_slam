#include "backend_optimization.h"
#include <Eigen/Core>
#include <sophus/so3.hpp>
#include <ceres/ceres.h>
#include "ceresfunc_ground.h"

void optimizeWithGroundConstraint(std::vector<KeyFrame>& keyframes) {
    using namespace std;
    int N = keyframes.size();
    if (N < 2) return;

    // 1. 关键帧SE3参数初始化（从T_w_l变换得到[tx,ty,tz,so3]）
    std::vector<Eigen::Matrix<double,6,1>, Eigen::aligned_allocator<Eigen::Matrix<double,6,1>>> pose_params(N);
    for (int i=0; i<N; ++i) {
        Eigen::Vector3d t = keyframes[i].pose.block<3,1>(0,3);
        Eigen::Matrix3d R = keyframes[i].pose.block<3,3>(0,0);
        Sophus::SO3d so3(R);
        pose_params[i].head<3>() = t;
        pose_params[i].tail<3>() = so3.log();
    }

    // 2. 初始化全局地面（可用第一个关键帧的平面）
    Cost_NavState_PR_Ground::init_ground_plane_coeff = keyframes[0].ground_plane_coeff;

    // 3. 构建ceres问题
    ceres::Problem problem;
    for (int i=0; i<N; ++i) {
        problem.AddParameterBlock(pose_params[i].data(), 6);
        problem.AddResidualBlock(
            Cost_NavState_PR_Ground::Create(keyframes[i].ground_plane_coeff),
            nullptr, pose_params[i].data()
        );
    }

    // 4. 优化设置
    ceres::Solver::Options options;
    options.max_num_iterations = 10;
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 5. 优化结果回填到关键帧
    for (int i=0; i<N; ++i) {
        Eigen::Vector3d t = pose_params[i].head<3>();
        Sophus::SO3d so3 = Sophus::SO3d::exp(pose_params[i].tail<3>());
        keyframes[i].pose.setIdentity();
        keyframes[i].pose.block<3,3>(0,0) = so3.matrix();
        keyframes[i].pose.block<3,1>(0,3) = t;
    }
}