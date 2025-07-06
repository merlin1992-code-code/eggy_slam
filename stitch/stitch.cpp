/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-07-06 22:35:42
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-06 22:35:44
 */
#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>

using CloudType = pcl::PointCloud<pcl::PointXYZINormal>;

struct Pose {
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
};

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " poses.yaml pcd_dir [output.pcd]" << std::endl;
        return 1;
    }
    std::string pose_yaml = argv[1];
    std::string pcd_dir = argv[2];
    std::string output_pcd = argc > 3 ? argv[3] : "global_map.pcd";

    // 1. 读取poses.yaml
    YAML::Node poses = YAML::LoadFile(pose_yaml);

    CloudType::Ptr global_map(new CloudType);

    for (auto it = poses.begin(); it != poses.end(); ++it) {
        std::string file_name = it->first.as<std::string>();
        auto node = it->second;
        auto pos = node["pos"];
        auto rot = node["rot"];
        Eigen::Vector3d t(pos[0].as<double>(), pos[1].as<double>(), pos[2].as<double>());
        Eigen::Quaterniond q(rot[3].as<double>(), rot[0].as<double>(), rot[1].as<double>(), rot[2].as<double>());

        // 2. 读取点云
        CloudType::Ptr cloud(new CloudType);
        std::string pcd_path = pcd_dir + "/" + file_name;
        if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(pcd_path, *cloud) == -1) {
            std::cerr << "Failed to load " << pcd_path << std::endl;
            continue;
        }

        // 3. 变换到世界坐标系
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = q.toRotationMatrix();
        T.block<3,1>(0,3) = t;
        CloudType::Ptr cloud_world(new CloudType);
        pcl::transformPointCloud(*cloud, *cloud_world, T.cast<float>());

        // 4. 拼接
        *global_map += *cloud_world;
    }

    // 5. 保存全局地图
    pcl::io::savePCDFileBinary(output_pcd, *global_map);
    std::cout << "Saved global map: " << output_pcd << std::endl;
    return 0;
}