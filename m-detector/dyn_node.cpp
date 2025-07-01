/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-06-28 21:18:18
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-01 15:14:10
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
  // 只用 config_path 初始化
  if (!std::filesystem::exists(config_path))
  {
    std::cerr << "Config file does not exist: " << config_path << std::endl;
    exit(EXIT_FAILURE);
  }

  node_ = load_yaml(config_path);
  m_cfg_ = node_["M_detector_cfg"].as<std::string>();
  root_dir_ = node_["root_dir"].as<std::string>();
  output_dir_ = node_["output_dir"].as<std::string>();
  ensure_out_dir(output_dir_);

  processor_.loadGPSData(root_dir_ + "rtk_gps.txt");
  processor_.loadOdomData(root_dir_ + "rtk_odom.txt");
  time_odom_map_ = processor_.processSyncFile(root_dir_ + "sync_time_list.txt");

  nh_ = std::make_shared<NodeHandle>(m_cfg_);
  DynObjFilt_ = std::make_shared<DynObjFilter>();
  DynObjFilt_->init(*nh_);
}

void DynNode::execute()
{
  cur_rot = Eigen::Matrix3d::Identity();
  cur_pos = Eigen::Vector3d::Zero();
  cur_time = 0;

  std::string pcd_dir = root_dir_ + "pcd/middle/";
  DIR *dirr = opendir(pcd_dir.c_str());
  if (!dirr)
  {
    std::cout << "Cannot open directory " << pcd_dir << std::endl;
    return;
  }
  std::deque<std::string> files;
  struct dirent *entry;
  while ((entry = readdir(dirr)) != NULL)
  {
    if (strcmp(entry->d_name, ".") && strcmp(entry->d_name, "..") &&
        strstr(entry->d_name, ".pcd") != NULL)
    {
      files.push_back(entry->d_name);
    }
  }
  closedir(dirr);
  sort(files.begin(), files.end());

  std::string file_name;
  while (!files.empty())
  {
    file_name = files.front();
    files.pop_front();

    std::cout << "file_name: " << file_name << std::endl;
    std::string file_path = pcd_dir + file_name;
    std::cout << "file_path: " << file_path << std::endl;
    if (rs128_handler(file_path, cur_pc) == false)
    {
      std::cout << "Failed to load point cloud from " << file_path << std::endl;
      continue;
    }
    unsigned long long time = std::stoull(file_name.substr(0, file_name.size() - 4));
    std::vector<double> odom = time_odom_map_[time];
    cur_rot = Eigen::Quaterniond(odom[6], odom[3], odom[4], odom[5]).toRotationMatrix();
    cur_pos = gps2enu(odom[0], odom[1], odom[2]);
    cur_time = time / 1e9;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "cur_pos: " << cur_pos.transpose() << std::endl;
    std::cout << "cur_rot: " << cur_rot << std::endl;
    std::cout << "cur_time: " << cur_time << std::endl;
    DynObjFilt_->filter(cur_pc, cur_rot, cur_pos, cur_time);
    DynObjFilt_->publish_dyn(output_dir_, file_name);
  }
}

void DynNode::execute_odom(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, M3D &rot, V3D &pos, double scan_end_time)
{
  DynObjFilt_->filter(cloud, rot, pos, scan_end_time);
  std::cout << std::fixed << std::setprecision(6);
  std::cout << "cur_pos: " << pos.transpose() << std::endl;
  std::cout << "cur_rot: " << rot << std::endl;
  std::cout << "cur_time: " << scan_end_time << std::endl;
  unsigned long long scan_time_int = static_cast<unsigned long long>(scan_end_time);
  std::string scan_file_name = std::to_string(scan_time_int) + ".pcd";
  DynObjFilt_->publish_dyn(output_dir_, scan_file_name);
}