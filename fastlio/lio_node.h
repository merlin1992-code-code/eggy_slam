/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-06-24 09:43:15
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-08 12:56:12
 */
#include <chrono>
#include <filesystem>
#include <iostream>
#include <algorithm>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>
#include <filesystem>
#include <boost/foreach.hpp>
#include <yaml-cpp/yaml.h>
#include "map_builder/commons.h"
#include "map_builder/map_builder.h"
#include "utils.h"
#include "viewer.h"

using namespace std;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

struct NodeConfig
{
    std::string rosbag = "data/RS.bag";
    std::string imu_topic = "/RS/imu";
    std::string lidar_topic = "/RS/lidar";
    std::string body_frame = "body";
    std::string world_frame = "lidar";
    bool print_time_cost = false;
};

struct StateData
{
    bool lidar_pushed = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    Path path;
};

class LIONode
{
public:
    explicit LIONode(const std::string &config_path);
    virtual void init_buffer();
    virtual void execute();
    CloudType::Ptr getLidarCloud();
    CloudType::Ptr getBodyCloud();
    CloudType::Ptr getWorldCloud();
    M3D getRWL();
    V3D getTWL();
    double scan_start_time();
    double scan_end_time();
    void debug_pose();
    std::vector<sensor_msgs::PointCloud2Ptr> getLidarVec() const { return lidar_vec; }
    std::vector<sensor_msgs::ImuPtr> getImuVec() const { return imu_vec; }
    void imuCB(const sensor_msgs::ImuConstPtr msg);
    void lidarCB(const sensor_msgs::PointCloud2Ptr msg);
    bool syncPackage();
    void processPackage();
    bool checkMappingStatus() const;
    std::vector<sensor_msgs::ImuPtr> get_imu_vec() const;
    void loadParameters(const std::string &config_path = "config.yaml");

public:
    YAML::Node config;
    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    std::vector<sensor_msgs::PointCloud2Ptr> lidar_vec;
    std::vector<sensor_msgs::ImuPtr> imu_vec;
    std::shared_ptr<CloudType> global_map{new CloudType};
    CloudType::Ptr body_cloud_{new CloudType};
    CloudType::Ptr world_cloud_{new CloudType};
    M3D r_wl_;
    V3D t_wl_;
    CloudViewer cloud_viewer;
};

class LIONodeSecond : public LIONode
{
public:
    LIONodeSecond(const std::string &config_path, const std::string &pcd_dir, const std::vector<sensor_msgs::ImuPtr> &imu_vec_in);
    void init_buffer() override;
    void lidarCB(size_t idx);
    void execute() override;

private:
    std::string pcd_dir_;
    std::vector<std::pair<double, CloudType::Ptr>> lidar_vec; // 直接存时间戳和点云
};