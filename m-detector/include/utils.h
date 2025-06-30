#ifndef UTILS_H
#define UTILS_H
#pragma once
#include <fstream>
#include <sstream>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

using namespace std;

struct GPSData
{
  double longitude;
  double latitude;
  double altitude;
};

struct OdomData
{
  double o_x;
  double o_y;
  double o_z;
  double o_w;
};

class RTKDataProcessor
{
public:
  void loadGPSData(const std::string &filename)
  {
    std::ifstream gpsFile(filename);
    std::string line;
    getline(gpsFile, line); // Skip header

    while (getline(gpsFile, line))
    {
      std::istringstream iss(line);
      unsigned long long timestamp;
      double lon, lat, alt;
      int status;
      iss >> timestamp >> lon >> lat >> alt >> status;
      gpsMap_[timestamp] = {lon, lat, alt};
    }
  }

  void loadOdomData(const std::string &filename)
  {
    std::ifstream odomFile(filename);
    std::string line;
    getline(odomFile, line); // Skip header

    while (getline(odomFile, line))
    {
      std::istringstream iss(line);
      unsigned long long timestamp;
      double vel_x, vel_y, vel_z, o_x, o_y, o_z, o_w;
      iss >> timestamp >> vel_x >> vel_y >> vel_z >> o_x >> o_y >> o_z >> o_w;
      odomMap_[timestamp] = {o_x, o_y, o_z, o_w};
    }
  }

  std::unordered_map<unsigned long long, std::vector<double>> processSyncFile(
      const std::string &filename)
  {
    std::unordered_map<unsigned long long, std::vector<double>> result;
    std::ifstream syncFile(filename);
    std::string line;
    getline(syncFile, line);

    int middleIdx = -1, gpsIdx = -1, odomIdx = -1;
    parseHeader(line, middleIdx, gpsIdx, odomIdx);

    while (getline(syncFile, line))
    {
      std::istringstream iss(line);
      std::vector<unsigned long long> timestamps;
      unsigned long long ts;

      while (iss >> ts)
      {
        timestamps.push_back(ts);
      }

      unsigned long long middleTS = timestamps[middleIdx];
      unsigned long long gpsTS = timestamps[gpsIdx];
      unsigned long long odomTS = timestamps[odomIdx];

      if (gpsMap_.count(gpsTS) && odomMap_.count(odomTS))
      {
        auto &gps = gpsMap_[gpsTS];
        auto &odom = odomMap_[odomTS];
        // printf("%llu %.6f %.6f %.6f %.6f %.6f %.6f\n", middleTS,
        // gps.longitude,
        //        gps.latitude, odom.o_x, odom.o_y, odom.o_z, odom.o_w);

        result[middleTS] = {
            gps.longitude, // longitude
            gps.latitude,  // latitude
            gps.altitude,  // latitude
            odom.o_x,      // o_x
            odom.o_y,      // o_y
            odom.o_z,      // o_z
            odom.o_w       // o_w
        };
      }
    }
    return result;
  }

private:
  std::unordered_map<unsigned long long, GPSData> gpsMap_;
  std::unordered_map<unsigned long long, OdomData> odomMap_;

  void parseHeader(const std::string &line, int &middleIdx, int &gpsIdx,
                   int &odomIdx)
  {
    std::istringstream titleStream(line.substr(1));
    std::vector<std::string> headers;
    std::string header;

    while (titleStream >> header)
    {
      headers.push_back(header);
      if (header == "/middle")
        middleIdx = headers.size() - 1;
      else if (header == "/rtk_gps")
        gpsIdx = headers.size() - 1;
      else if (header == "/rtk_odom")
        odomIdx = headers.size() - 1;
    }
  }
};

namespace rs128
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace rs128
POINT_CLOUD_REGISTER_POINT_STRUCT(rs128::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))

inline bool rs128_handler(const std::string &pcd_path, pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud)
{
  float blind = 0.4f;
  float max_range = 250.0f;
  int N_SCANS = 128;
  int point_filter_num = 1;

  cloud->clear();
  int num_invalid_pt = 0;
  int num_out_of_range_pt = 0;
  pcl::PointCloud<rs128::Point> pl_orig;
  if (pcl::io::loadPCDFile<rs128::Point>(pcd_path, pl_orig) == -1)
  {
    std::cerr << "无法读取RS128 PCD文件: " << pcd_path << std::endl;
    return false;
  }
  int plsize = pl_orig.points.size();
  if (plsize == 0)
    return false;
  cloud->reserve(plsize);

  for (int i = 0; i < plsize; i++)
  {
    if (i % point_filter_num != 0)
      continue;

    float x = pl_orig.points[i].x;
    float y = pl_orig.points[i].y;
    float z = pl_orig.points[i].z;
    if (std::isnan(x) || std::isnan(y) || std::isnan(z))
    {
      num_invalid_pt += 1;
      continue;
    }
    float r2 = x * x + y * y + z * z;
    if (r2 < blind * blind || r2 > max_range * max_range)
    {
      num_out_of_range_pt += 1;
      continue;
    }

    pcl::PointXYZINormal added_pt;
    added_pt.x = x;
    added_pt.y = y;
    added_pt.z = z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.curvature = static_cast<float>(pl_orig.points[i].timestamp);
    cloud->push_back(added_pt);
  }

  std::cout << "Converted PointCloud2 to PCL cloud with " << plsize << " points, "
            << num_invalid_pt << " invalid points filtered. "
            << num_out_of_range_pt << " out of range points filtered. " << std::endl;
  return true;
}

struct AppConfig
{
  string config_path = "config.yaml";
};

inline void parse_args(int argc, char **argv, AppConfig &cfg)
{
  for (int i = 1; i < argc; ++i)
  {
    std::string arg = argv[i];
    if (arg.rfind("--cfg=", 0) == 0)
      cfg.config_path = arg.substr(6);
  }
}

inline bool check_and_collect_files(AppConfig &cfg)
{
  struct stat buffer;
  if (stat(cfg.config_path.c_str(), &buffer) != 0)
  {
    std::cerr << "配置文件不存在: " << cfg.config_path << std::endl;
    return false;
  }
  return true;
}

inline YAML::Node load_yaml(const std::string &yaml_file)
{
  YAML::Node config;
  if (!std::filesystem::exists(yaml_file))
  {
    std::cerr << "[load_yaml] YAML 文件不存在: " << yaml_file << std::endl;
    throw std::runtime_error("YAML file not found");
  }
  try
  {
    config = YAML::LoadFile(yaml_file);
  }
  catch (const std::exception &e)
  {
    std::cerr << "[load_yaml] 读取 YAML 文件失败: " << yaml_file << "\n"
              << "原因: " << e.what() << std::endl;
    throw;
  }
  return config;
}

inline void ensure_out_dir(const string &out_dir)
{
  if (!std::filesystem::exists(out_dir))
  {
    std::filesystem::create_directories(out_dir);
  }
}

#endif // UTILS_H