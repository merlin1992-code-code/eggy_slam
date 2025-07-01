/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-06-28 21:18:18
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-01 11:39:53
 */
#include <iostream>
#include <dirent.h>

#include <proj.h>
#include "m_utils.h"
#include "DynObjFilter.h"

int main(int argc, char *argv[])
{

  AppConfig cfg;
  parse_args(argc, argv, cfg);
  if (!check_and_collect_files(cfg))
    return -1;
  std::cout << "Config file: " << cfg.config_path << std::endl;
  YAML::Node node = load_yaml(cfg.config_path);
  std::string m_cfg = node["M_detector_cfg"].as<std::string>();
  std::string root_dir = node["root_dir"].as<std::string>();
  std::string output_dir = node["output_dir"].as<std::string>();
  ensure_out_dir(output_dir);

  RTKDataProcessor processor;
  std::unordered_map<unsigned long long, std::vector<double>> time_odom_map;
  processor.loadGPSData(root_dir + "rtk_gps.txt");
  processor.loadOdomData(root_dir + "rtk_odom.txt");
  time_odom_map = processor.processSyncFile(root_dir + "sync_time_list.txt");

  NodeHandle nh(m_cfg);
  DynObjFilter *DynObjFilt = new DynObjFilter();

  DynObjFilt->init(nh);

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cur_pc(new pcl::PointCloud<pcl::PointXYZINormal>());
  // std::shared_ptr<PointCloudXYZI> cur_pc(new PointCloudXYZI());
  M3D cur_rot = Eigen::Matrix3d::Identity();
  V3D cur_pos = Eigen::Vector3d::Zero();
  double cur_time = 0;

  std::string pcd_dir = root_dir + "pcd/middle/";
  DIR *dirr = opendir(pcd_dir.c_str());
  if (!dirr)
  {
    std::cout << "Cannot open directory " << pcd_dir << std::endl;
    return 0;
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
    unsigned long long time =
        std::stod(file_name.substr(0, file_name.size() - 4));
    std::vector<double> odom = time_odom_map[time];
    cur_rot = Eigen::Quaterniond(odom[6], odom[3], odom[4], odom[5])
                  .toRotationMatrix();

    cur_pos = gps2enu(odom[0], odom[1], odom[2]);
    cur_time = time / 1e9;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "cur_pos: " << cur_pos.transpose() << std::endl;
    std::cout << "cur_rot: " << cur_rot << std::endl;
    std::cout << "cur_time: " << cur_time << std::endl;
    DynObjFilt->filter(cur_pc, cur_rot, cur_pos, cur_time);
    DynObjFilt->publish_dyn(output_dir, file_name);
  }
  return 0;
}
