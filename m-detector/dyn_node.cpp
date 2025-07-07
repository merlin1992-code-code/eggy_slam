/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-07-04 16:08:03
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-07 10:19:23
 */
/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-06-28 21:18:18
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-04 21:50:04
 */
#include "dyn_node.h"
#include <iostream>
#include <dirent.h>
#include <iomanip>

DynNode::DynNode(const std::string &config_path)
    : cur_pc(new pcl::PointCloud<pcl::PointXYZINormal>()),
      cur_rot(Eigen::Matrix3d::Identity()),
      cur_pos(Eigen::Vector3d::Zero()),
      cur_time(0.0)
{
  if (!std::filesystem::exists(config_path))
  {
    std::cerr << "Config file does not exist: " << config_path << std::endl;
    exit(EXIT_FAILURE);
  }

  node_ = load_yaml(config_path);
  m_cfg_ = node_["M_detector_cfg"].as<std::string>();
  root_dir_ = node_["root_dir"].as<std::string>();
  output_dir_ = node_["output_dir"].as<std::string>();
  output_fusion_dir_ = node_["output_fusion_dir"].as<std::string>();
  ensure_out_dir(output_dir_);
  ensure_out_dir(output_fusion_dir_);
  nh_ = std::make_shared<NodeHandle>(m_cfg_);
  DynObjFilt_ = std::make_shared<DynObjFilter>();
  DynObjFilt_->init(*nh_);
}

void DynNode::saveTrajectory(const std::string &out_dir)
{
  std::ofstream traj_file(out_dir + "/trajectory.csv");
  traj_file << "x,y,z,rot00,rot01,rot02,rot10,rot11,rot12,rot20,rot21,rot22\n";
  for (const auto &rec : pose_records_)
  {
    traj_file << rec.pos.x() << "," << rec.pos.y() << "," << rec.pos.z();
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        traj_file << "," << rec.rot(i, j);
    traj_file << "\n";
  }
  traj_file.close();
}

void DynNode::execute(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, M3D &cur_rot, V3D &cur_pos, double scan_end_time)
{
  DynObjFilt_->filter(cloud, cur_rot, cur_pos, scan_end_time);
  std::cout << std::fixed << std::setprecision(6);
  std::cout << "cur_pos: " << std::endl;
  std::cout << cur_pos.transpose() << std::endl;
  std::cout << "cur_rot: " << std::endl;
  std::cout << cur_rot << std::endl;
  std::cout << "cur_end_time: " << scan_end_time << std::endl;
  unsigned long long scan_end_time_int = static_cast<unsigned long long>(scan_end_time * 1e9);
  std::string scan_file_name = std::to_string(scan_end_time_int) + ".pcd";
  DynObjFilt_->publish_dyn(output_fusion_dir_, scan_file_name, cur_rot, cur_pos);
  pose_records_.push_back({cur_pos, cur_rot});
}

std::string DynNode::getStdPcdDir()
{
  return output_fusion_dir_ + "/std_cluster";
}