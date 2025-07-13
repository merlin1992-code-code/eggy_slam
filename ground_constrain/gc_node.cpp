/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-07-10 10:36:36
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-13 22:37:38
 */
#include <iostream>
#include <vector>
#include <string>
#include <dirent.h>
#include <filesystem>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <nlohmann/json.hpp>
#include "KeyFrame.h"
#include "extract_fit_ground_plane.h"
#include "ground_plane_utils.h"
#include "backend_optimization.h"
#include "make_keyframes.h"
#include "projection_param.h"
#include "dipgseg.h"

#define SAVE_GROUND_PCD 0

int main(int argc, char **argv)
{
    if (argc < 4)
    {
        std::cerr << "Usage: " << argv[0] << " <pcd_dir> <pose.json> <config.yaml>" << std::endl;
        return -1;
    }
    std::string pcd_dir = argv[1];
    std::string pose_json_path = argv[2];
    std::string config_path = argv[3];


    // 1. 读取 pose.json
    std::ifstream fin(pose_json_path);
    if (!fin.is_open())
    {
        std::cerr << "Failed to open pose json: " << pose_json_path << std::endl;
        return -1;
    }

    LidarParam param;
    bool success = load_lidar_param(config_path, param);
    if (!success)
    {
        std::cerr << "Failed to load lidar parameters from: " << config_path << std::endl;
        return -1;
    }
    std::string ground_pcd_dir = param.ground_pcd_dir;
    std::filesystem::create_directory(ground_pcd_dir);
    auto dipgseg = std::make_shared<DIPGSEG::Dipgseg>(param);
    nlohmann::json poses_json;
    fin >> poses_json;

    std::vector<std::string> pcd_files;
    std::vector<Eigen::Matrix4d> poses;
    std::vector<KeyFrame> keyframes;

    for (auto it = poses_json.begin(); it != poses_json.end(); ++it)
    {
        std::string pcd_file = it.key();
        auto pose_node = it.value()["pose"];
        auto rot_node = pose_node["rotation"];
        auto trans_node = pose_node["translation"];
        Eigen::Quaterniond q(
            rot_node["w"].get<double>(),
            rot_node["x"].get<double>(),
            rot_node["y"].get<double>(),
            rot_node["z"].get<double>());
        Eigen::Vector3d t(
            trans_node["x"].get<double>(),
            trans_node["y"].get<double>(),
            trans_node["z"].get<double>());
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
        T.block<3, 1>(0, 3) = t;

        std::string full_pcd_path = (std::filesystem::path(pcd_dir) / pcd_file).string();
        pcl::PointCloud<pcl::PointXYZINormal> cloud_in;
        if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(full_pcd_path, cloud_in) == -1)
        {
            std::cerr << "Failed to load: " << full_pcd_path << std::endl;
            continue;
        }
        // 如果点云没有法向量，需要估算
        if (cloud_in.points[0].normal_x == cloud_in.points[0].normal_y &&
            cloud_in.points[0].normal_x == cloud_in.points[0].normal_z &&
            cloud_in.points[0].normal_x == 0)
        {
            pcl::NormalEstimation<pcl::PointXYZINormal, pcl::PointXYZINormal> ne;
            ne.setInputCloud(cloud_in.makeShared());
            pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZINormal>());
            ne.setSearchMethod(tree);
            ne.setKSearch(10);
            ne.compute(cloud_in);
        }
        pcl::PointCloud<pcl::PointXYZINormal> ground_cloud, cloud_non_ground;
        std::string ground_pcd_path = ground_pcd_dir + "/" + pcd_file;
        dipgseg->segment_ground(cloud_in, ground_cloud, cloud_non_ground);
        //pcl::io::savePCDFileBinary(ground_pcd_path, ground_cloud);
        KeyFrame kf;
        *kf.filtered_cloud = ground_cloud;
        kf.pose = T;
        Eigen::Vector4f plane_coeff = get_plane_coeffs(ground_cloud);
        kf.ground_plane_coeff = plane_coeff;
        keyframes.push_back(kf);
#if SAVE_GROUND_PCD
        pcl::PointCloud<pcl::PointXYZINormal> ground_cloud_ransac;
        ground_filtered(ground_cloud, plane_coeff, ground_cloud_ransac, 0.08f);
        if (ground_cloud_ransac.size() < 10)
        {
            std::cerr << "Ground cloud too small after filtering: " << ground_cloud_ransac.size() << std::endl;
            continue;
        }
        pcl::io::savePCDFileBinary(ground_pcd_path, ground_cloud_ransac);
#endif 
    }

    if (keyframes.size() < 2)
    {
        std::cerr << "Not enough valid keyframes!" << std::endl;
        return -1;
    }

    // 后端地面约束优化
    optimizeWithGroundConstraint(keyframes);

    //输出优化后轨迹
    for (size_t i = 0; i < keyframes.size(); ++i)
    {
        std::cout << "Frame " << i << " pose:\n"
                  << keyframes[i].pose << std::endl;
    }

    // 保存优化后的 pose 到 json
    nlohmann::json refined_json;
    size_t idx = 0;
    for (auto it = poses_json.begin(); it != poses_json.end(); ++it, ++idx)
    {
        if (idx >= keyframes.size())
            break;
        const std::string &pcd_file = it.key();
        const Eigen::Matrix4d &T = keyframes[idx].pose;
        Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        Eigen::Quaterniond q(R);
        Eigen::Vector3d t = T.block<3, 1>(0, 3);

        refined_json[pcd_file]["pose"]["rotation"]["w"] = q.w();
        refined_json[pcd_file]["pose"]["rotation"]["x"] = q.x();
        refined_json[pcd_file]["pose"]["rotation"]["y"] = q.y();
        refined_json[pcd_file]["pose"]["rotation"]["z"] = q.z();
        refined_json[pcd_file]["pose"]["translation"]["x"] = t.x();
        refined_json[pcd_file]["pose"]["translation"]["y"] = t.y();
        refined_json[pcd_file]["pose"]["translation"]["z"] = t.z();
    }

    std::ofstream fout("gc_refined_poses.json");
    fout << refined_json.dump(4);
    fout.close();

    return 0;
}