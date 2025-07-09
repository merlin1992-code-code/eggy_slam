/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-07-09 11:04:30
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-09 13:10:49
 */
#include <iostream>
#include <fstream>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include "hba/blam.h"
#include "hba/hba.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;


void fromStr(const std::string &str, std::string &file_name, Pose &pose)
{
    std::stringstream ss(str);
    std::vector<std::string> tokens;
    std::string token;
    while (std::getline(ss, token, ' '))
    {
        tokens.push_back(token);
    }
    assert(tokens.size() == 8);
    file_name = tokens[0];
    pose.t = V3D(std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]));
    pose.r = Eigen::Quaterniond(std::stod(tokens[4]), std::stod(tokens[5]), std::stod(tokens[6]), std::stod(tokens[7])).normalized().toRotationMatrix();
}

int main(int argc, char *argv[])
{
    pcl::PCDReader reader;
    HBAConfig config;

    double scan_resolution = 0.5;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setLeafSize(scan_resolution, scan_resolution, scan_resolution);

    HBA hba(config);

    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " poses.yaml|poses.json pcd_dir [output.pcd]" << std::endl;
        return 1;
    }
    std::string pose_file = argv[1];
    std::string pcd_dir = argv[2];
    std::string output_pcd = argc > 3 ? argv[3] : "refine_map.pcd";


    std::cout << "hba excute" << std::endl;
    std::ifstream fin(pose_file);
    if (!fin.is_open())
    {
        std::cerr << "Failed to open " << pose_file << std::endl;
        return 1;
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

        Pose pose;
        pose.t = t;
        pose.r = q.normalized().toRotationMatrix();
        std::string pcd_path = pcd_dir + "/" + file_name;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *cloud) == -1)
        {
            std::cerr << "Failed to load " << pcd_path << std::endl;
            continue;
        }
        voxel_grid.setInputCloud(cloud);
        voxel_grid.filter(*cloud);
        hba.insert(cloud, pose, file_name);
    }
    pcl::io::savePCDFileBinary("before_opt.pcd", *hba.getMapPoints());
    hba.optimize();
    pcl::io::savePCDFileBinary("after_opt.pcd", *hba.getMapPoints());
    hba.writeJsonPoses("refined_poses.json");
    return 0;
}