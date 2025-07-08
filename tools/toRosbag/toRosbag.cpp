/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-06-23 10:43:07
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-07 22:18:13
 */

#include "toRosbag.h"

namespace fs = std::filesystem;
const double MIN_VALID_TIMESTAMP = 1e8;

// rs128::Point 特化
template <>
void pointCloudToMsg<rs128::Point>(const pcl::PointCloud<rs128::Point> &cloud,
                                   sensor_msgs::PointCloud2 &msg)
{
  msg.header.frame_id = "your_frame";
  msg.height = 1;
  msg.width = cloud.size();
  msg.is_bigendian = false;
  msg.is_dense = cloud.is_dense;
  msg.point_step = 24; // 4*4 + 8
  msg.row_step = msg.point_step * cloud.size();

  msg.fields.resize(5);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 12;
  msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[3].count = 1;
  msg.fields[4].name = "timestamp";
  msg.fields[4].offset = 16;
  msg.fields[4].datatype = sensor_msgs::PointField::FLOAT64;
  msg.fields[4].count = 1;

  msg.data.resize(msg.row_step);

  size_t nan_count = 0;
  for (size_t i = 0; i < cloud.size(); ++i)
  {
    if (std::isnan(cloud.points[i].x) || std::isnan(cloud.points[i].y) ||
        std::isnan(cloud.points[i].z))
    {
      nan_count++;
      continue;
    }
    uint8_t *ptr = &msg.data[i * msg.point_step];
    *reinterpret_cast<float *>(ptr + 0) = cloud.points[i].x;
    *reinterpret_cast<float *>(ptr + 4) = cloud.points[i].y;
    *reinterpret_cast<float *>(ptr + 8) = cloud.points[i].z;
    *reinterpret_cast<float *>(ptr + 12) = cloud.points[i].intensity;
    // 第一个点 1717465828.100000858 start time 但是 lidar 的pcd 命名是 endtime
    // 1717465828.200002816
    *reinterpret_cast<double *>(ptr + 16) = cloud.points[i].timestamp;
  }
  if (nan_count > 0)
  {
    std::cout << "本帧点云中有 NaN 点数量: " << nan_count << std::endl;
  }
}

template <typename PointT>
void writePcdToBag(const std::string &pcd_folder, rosbag::Bag &bag,
                   const std::string &point_topic)
{
  std::vector<std::string> pcd_files;
  for (const auto &entry : fs::directory_iterator(pcd_folder))
  {
    if (entry.path().extension() == ".pcd")
    {
      pcd_files.push_back(entry.path().string());
    }
  }
  std::sort(pcd_files.begin(), pcd_files.end());

  for (const auto &file : pcd_files)
  {
    pcl::PointCloud<PointT> cloud;
    if (pcl::io::loadPCDFile<PointT>(file, cloud) == -1)
    {
      std::cerr << "Failed to load " << file << std::endl;
      continue;
    }
    if (cloud.points.empty())
    {
      std::cerr << "点云为空，跳过: " << file << std::endl;
      continue;
    }

    // 对点按 timestamp 升序排序
    std::sort(cloud.points.begin(), cloud.points.end(),
              [](const PointT &a, const PointT &b)
              {
                return a.timestamp < b.timestamp;
              });

    // 获取第一个有效点的 timestamp
    double start_time = 0;
    bool found_valid = false;
    
    for (const auto &pt : cloud.points)
    {
      std::cout << "pt: ts=" << pt.timestamp << std::endl;
      if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z) && pt.timestamp > MIN_VALID_TIMESTAMP)
      {
        std::cout << "DEBUG: will write " << file << std::endl;
        start_time = pt.timestamp;
        std::cout << "IIINNN start_time =" << start_time << std::endl;
        found_valid = true;
        break;
      }
    }
    if (!found_valid || start_time <= 0 || std::isnan(start_time))
    {
      std::cerr << "本帧无有效点或时间戳无效，跳过: " << file << std::endl;
      continue;
    }

    std::string filename = fs::path(file).stem().string();

    uint64_t ns = static_cast<uint64_t>(start_time * 1e9);
    std::cout << std::fixed << std::setprecision(9) << ns << "  " << filename << std::endl;

    // pcl::io::savePCDFileASCII(filename + ".pcd", cloud);
    sensor_msgs::PointCloud2 msg;
    msg.header.stamp.sec = static_cast<uint32_t>(ns / 1000000000ULL);
    msg.header.stamp.nsec = static_cast<uint32_t>(ns % 1000000000ULL);
    pointCloudToMsg(cloud, msg);
    std::cout << "PointCloud2 msg: " << msg.header.stamp.sec << "."
              << std::setfill('0') << std::setw(9) << msg.header.stamp.nsec
              << " for file " << file << std::endl;
    bag.write(point_topic, msg.header.stamp, msg);
    std::cout << "Wrote: " << file << std::endl;
  }
  std::cout << "Finished writing PCD files to bag." << std::endl;
  std::cout << "Total PCD files written: " << pcd_files.size() << std::endl;
}

void writeImuToBag(const std::string &imu_txt, rosbag::Bag &bag,
                   const std::string &imu_topic)
{
  std::ifstream fin(imu_txt);
  std::string line;
  int line_count = 0;
  while (std::getline(fin, line))
  {
    if (line.empty() || line[0] == '#')
      continue;

    std::istringstream iss(line);
    uint64_t ns;
    double ax, ay, az, gx, gy, gz, qx, qy, qz, qw;
    iss >> ns >> ax >> ay >> az >> gx >> gy >> gz >> qx >> qy >> qz >> qw;
    // std::cout << "IMU data: " << ns << " " << ax << " " << ay << " "
    //           << az << " " << gx << " " << gy << " " << gz << " "
    //           << qx << " " << qy << " " << qz << " " << qw << std::endl;
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp.sec = static_cast<uint32_t>(ns / 1000000000ULL);
    imu_msg.header.stamp.nsec = static_cast<uint32_t>(ns % 1000000000ULL);
    imu_msg.header.frame_id = "imu_link";
    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;
    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;
    imu_msg.orientation.x = qx;
    imu_msg.orientation.y = qy;
    imu_msg.orientation.z = qz;
    imu_msg.orientation.w = qw;
    bag.write(imu_topic, imu_msg.header.stamp, imu_msg);
    line_count++;
  }
  std::cout << "Finished writing IMU data to bag." << std::endl;
  std::cout << "Total IMU lines written: " << line_count << std::endl;
}

template void writePcdToBag<rs128::Point>(const std::string &, rosbag::Bag &, const std::string &);

// 获取排序后的所有pcd文件
std::vector<std::string> get_sorted_pcd_files(const std::string &pcd_folder)
{
  std::vector<std::string> pcd_files;
  for (const auto &entry : fs::directory_iterator(pcd_folder))
  {
    if (entry.path().extension() == ".pcd")
      pcd_files.push_back(entry.path().string());
  }
  std::sort(pcd_files.begin(), pcd_files.end());
  return pcd_files;
}

// 读取IMU所有行
std::vector<std::string> get_imu_lines(const std::string &imu_txt)
{
  std::vector<std::string> lines;
  std::ifstream fin(imu_txt);
  std::string line;
  while (std::getline(fin, line))
  {
    if (!line.empty() && line[0] != '#')
      lines.push_back(line);
  }
  return lines;
}

template <typename PointT>
void writeBatchBags(const std::string &pcd_folder, const std::string &imu_txt,
                    const std::string &bag_prefix,
                    const std::string &point_topic, const std::string &imu_topic,
                    size_t batch_size)
{
  auto pcd_files = get_sorted_pcd_files(pcd_folder);
  auto imu_lines = get_imu_lines(imu_txt);

  size_t total_pcd = pcd_files.size();
  size_t total_imu = imu_lines.size();
  size_t imu_per_batch = batch_size * 10;

  size_t imu_idx = 0;
  size_t batch_id = 0;
  for (size_t batch_start = 0; batch_start < total_pcd; batch_start += batch_size, ++batch_id)
  {
    size_t batch_end = std::min(batch_start + batch_size, total_pcd);
    std::string batch_bag_path = bag_prefix + "_batch_" + std::to_string(batch_id) + ".bag";
    std::cout << "Writing bag: " << batch_bag_path << " [" << batch_start << ", " << batch_end << ")" << std::endl;

    rosbag::Bag bag;
    bag.open(batch_bag_path, rosbag::bagmode::Write);

    // 写入点云
    for (size_t i = batch_start; i < batch_end; ++i)
    {
      const auto &file = pcd_files[i];
      pcl::PointCloud<PointT> cloud;
      if (pcl::io::loadPCDFile<PointT>(file, cloud) == -1)
      {
        std::cerr << "Failed to load " << file << std::endl;
        continue;
      }
      std::sort(cloud.points.begin(), cloud.points.end(),
                [](const PointT &a, const PointT &b)
                { return a.timestamp < b.timestamp; });

      double start_time = 0;
      bool found_valid = false;
      for (const auto &pt : cloud.points)
      {
        if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z) && pt.timestamp > 0)
        {
          start_time = pt.timestamp;
          found_valid = true;
          break;
        }
      }
      if (!found_valid)
      {
        std::cerr << "本帧无有效点或时间戳无效，跳过: " << file << std::endl;
        continue;
      }
      if (start_time <= 0)
      {
        std::cerr << "本帧无有效点，跳过: " << file << std::endl;
        continue;
      }

      uint64_t ns = static_cast<uint64_t>(start_time * 1e9);
      sensor_msgs::PointCloud2 msg;
      msg.header.stamp.sec = static_cast<uint32_t>(ns / 1000000000ULL);
      msg.header.stamp.nsec = static_cast<uint32_t>(ns % 1000000000ULL);
      pointCloudToMsg(cloud, msg);
      bag.write(point_topic, msg.header.stamp, msg);
    }

    // 写入IMU
    size_t imu_batch_end = std::min(imu_idx + imu_per_batch, total_imu);
    for (; imu_idx < imu_batch_end; ++imu_idx)
    {
      std::istringstream iss(imu_lines[imu_idx]);
      uint64_t ns;
      double ax, ay, az, gx, gy, gz, qx, qy, qz, qw;
      iss >> ns >> ax >> ay >> az >> gx >> gy >> gz >> qx >> qy >> qz >> qw;
      sensor_msgs::Imu imu_msg;
      imu_msg.header.stamp.sec = static_cast<uint32_t>(ns / 1000000000ULL);
      imu_msg.header.stamp.nsec = static_cast<uint32_t>(ns % 1000000000ULL);
      imu_msg.header.frame_id = "imu_link";
      imu_msg.linear_acceleration.x = ax;
      imu_msg.linear_acceleration.y = ay;
      imu_msg.linear_acceleration.z = az;
      imu_msg.angular_velocity.x = gx;
      imu_msg.angular_velocity.y = gy;
      imu_msg.angular_velocity.z = gz;
      imu_msg.orientation.x = qx;
      imu_msg.orientation.y = qy;
      imu_msg.orientation.z = qz;
      imu_msg.orientation.w = qw;
      bag.write(imu_topic, imu_msg.header.stamp, imu_msg);
    }

    bag.close();
    std::cout << "Batch bag written: " << batch_bag_path << std::endl;
  }
  std::cout << "All batch bags finished. Total pcd: " << total_pcd << ", total imu: " << imu_idx << std::endl;
}

// 模板实例化
template void writeBatchBags<rs128::Point>(const std::string &, const std::string &, const std::string &, const std::string &, const std::string &, size_t);