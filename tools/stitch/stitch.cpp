/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-07-06 22:35:42
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-07 14:42:02
 */
#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>

using CloudType = pcl::PointCloud<pcl::PointXYZINormal>;
using json = nlohmann::json;

bool ends_with(const std::string &str, const std::string &suffix)
{
    return str.size() >= suffix.size() &&
           str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// 读取 YAML 格式的 pose
void stitch_from_yaml(const std::string &pose_yaml, const std::string &pcd_dir, CloudType::Ptr global_map)
{
    YAML::Node poses = YAML::LoadFile(pose_yaml);
    for (auto it = poses.begin(); it != poses.end(); ++it)
    {
        std::string file_name = it->first.as<std::string>();
        auto node = it->second;
        auto pos = node["pos"];
        auto rot = node["rot"];
        Eigen::Vector3d t(pos[0].as<double>(), pos[1].as<double>(), pos[2].as<double>());
        Eigen::Quaterniond q(rot[3].as<double>(), rot[0].as<double>(), rot[1].as<double>(), rot[2].as<double>());

        CloudType::Ptr cloud(new CloudType);
        std::string pcd_path = pcd_dir + "/" + file_name;
        if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(pcd_path, *cloud) == -1)
        {
            std::cerr << "Failed to load " << pcd_path << std::endl;
            continue;
        }
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T.block<3, 1>(0, 3) = t;
        CloudType::Ptr cloud_world(new CloudType);
        pcl::transformPointCloud(*cloud, *cloud_world, T.cast<float>());
        *global_map += *cloud_world;
    }
}

// 读取 JSON 格式的 pose
void stitch_from_json(const std::string &pose_json, const std::string &pcd_dir, CloudType::Ptr global_map)
{
    std::ifstream fin(pose_json);
    if (!fin.is_open())
    {
        std::cerr << "Failed to open " << pose_json << std::endl;
        return;
    }
    json poses;
    fin >> poses;
    for (auto it = poses.begin(); it != poses.end(); ++it)
    {
        std::string file_name = it.key();
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

        CloudType::Ptr cloud(new CloudType);
        std::string pcd_path = pcd_dir + "/" + file_name;
        if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(pcd_path, *cloud) == -1)
        {
            std::cerr << "Failed to load " << pcd_path << std::endl;
            continue;
        }
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T.block<3, 1>(0, 3) = t;
        CloudType::Ptr cloud_world(new CloudType);
        pcl::transformPointCloud(*cloud, *cloud_world, T.cast<float>());
        *global_map += *cloud_world;
    }
}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " poses.yaml|poses.json pcd_dir [output.pcd]" << std::endl;
        return 1;
    }
    std::string pose_file = argv[1];
    std::string pcd_dir = argv[2];
    std::string output_pcd = argc > 3 ? argv[3] : "global_map.pcd";

    CloudType::Ptr global_map(new CloudType);

    if (ends_with(pose_file, ".yaml") || ends_with(pose_file, ".yml"))
    {
        stitch_from_yaml(pose_file, pcd_dir, global_map);
    }
    else if (ends_with(pose_file, ".json"))
    {
        stitch_from_json(pose_file, pcd_dir, global_map);
    }
    else
    {
        std::cerr << "Unsupported pose file format: " << pose_file << std::endl;
        return 1;
    }

    pcl::io::savePCDFileBinary(output_pcd, *global_map);
    std::cout << "Saved global map: " << output_pcd << std::endl;
    return 0;
}