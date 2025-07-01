/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-07-01 09:52:16
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-01 11:10:13
 */
#include "toRosbag.h"

namespace fs = std::filesystem;

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "用法: " << argv[0] << " config.yaml" << std::endl;
    return 1;
  }
  if (!fs::exists(argv[1])) {
    std::cerr << "配置文件不存在: " << argv[1] << std::endl;
    return 1;
  }
  YAML::Node config = YAML::LoadFile(argv[1]);
  std::string bag_path = config["bag_path"].as<std::string>();
  std::string pcd_folder = config["pcd_folder"].as<std::string>();
  if (!fs::exists(pcd_folder)) {
    std::cerr << "PCD文件夹不存在: " << pcd_folder << std::endl;
    return 1;
  }
  std::string point_topic = config["point_topic"].as<std::string>();
  std::string imu_txt = config["imu_txt"].as<std::string>();
  if (!fs::exists(imu_txt)) {
    std::cerr << "IMU文本文件不存在: " << imu_txt << std::endl;
    return 1;
  }
  std::string imu_topic = config["imu_topic"].as<std::string>();

  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Write);
  std::cout << "开始写入Rosbag: " << bag_path << std::endl;
  writePcdToBag<rs128::Point>(pcd_folder, bag, point_topic);
  writeImuToBag(imu_txt, bag, imu_topic);
  bag.close();
  return 0;
}