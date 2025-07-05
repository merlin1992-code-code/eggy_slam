/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-06-24 09:43:15
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-05 18:40:06
 */

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#include "lio_node.h"

using namespace std;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

LIONode::LIONode(const std::string &config_path)
{
    std::cout << "LIO Node Started" << std::endl;
    loadParameters(config_path);
    init_buffer();
    m_state_data.path.poses.clear();
    m_kf = std::make_shared<IESKF>();
    m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);
}

void LIONode::loadParameters(const std::string &config_path)
{
    config = YAML::LoadFile(config_path);
    m_node_config.rosbag = config["rosbag"].as<std::string>();
    m_node_config.imu_topic = config["imu_topic"].as<std::string>();
    m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
    m_node_config.body_frame = config["body_frame"].as<std::string>();
    m_node_config.world_frame = config["world_frame"].as<std::string>();
    m_node_config.print_time_cost = config["print_time_cost"].as<bool>();

    m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
    m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
    m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
    m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
    m_builder_config.map_resolution = config["map_resolution"].as<double>();
    m_builder_config.cube_len = config["cube_len"].as<double>();
    m_builder_config.det_range = config["det_range"].as<double>();
    m_builder_config.move_thresh = config["move_thresh"].as<double>();
    m_builder_config.na = config["na"].as<double>();
    m_builder_config.ng = config["ng"].as<double>();
    m_builder_config.nba = config["nba"].as<double>();
    m_builder_config.nbg = config["nbg"].as<double>();

    m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
    m_builder_config.near_search_num = config["near_search_num"].as<int>();
    m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
    m_builder_config.gravity_align = config["gravity_align"].as<bool>();
    m_builder_config.esti_il = config["esti_il"].as<bool>();
    std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
    std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
    m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
    m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6], r_il_vec[7], r_il_vec[8];
    m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();
    m_builder_config.map_save_interval = config["map_save_interval"].as<int>();
}

void LIONode::init_buffer()
{
    Utils::ParseRosbagToVectors(
        m_node_config.rosbag,
        m_node_config.lidar_topic,
        m_node_config.imu_topic,
        lidar_vec,
        imu_vec);
    if (lidar_vec.empty() || imu_vec.empty())
    {
        std::cerr << "Lidar or IMU data is empty, please check the rosbag file." << std::endl;
        exit(1);
    }
}

void LIONode::imuCB(const sensor_msgs::ImuConstPtr msg)
{
    double timestamp = msg->header.stamp.toNSec();
    if (timestamp < m_state_data.last_imu_time)
    {
        std::cerr << "IMU Message is out of order" << std::endl;
        std::deque<IMUData>().swap(m_state_data.imu_buffer);
    }
    m_state_data.imu_buffer.emplace_back(V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) * 10.0,
                                         V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                         timestamp);
    m_state_data.last_imu_time = timestamp;
}

void LIONode::lidarCB(const sensor_msgs::PointCloud2Ptr msg)
{
    std::cout << "lidarCB" << std::endl;
    CloudType::Ptr cloud = Utils::convertToPCL(msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range, m_builder_config.lidar_max_range);
    double timestamp = msg->header.stamp.toNSec();
    if (!cloud || cloud->empty())
    {
        std::cerr << "Warning: Converted cloud is empty!" << std::endl;
    }
    if (timestamp < m_state_data.last_lidar_time)
    {
        std::cerr << "Lidar Message is out of order" << std::endl;
        std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
    }
    m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
    m_state_data.last_lidar_time = timestamp;
}

bool LIONode::syncPackage()
{
    if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
        return false;
    if (!m_state_data.lidar_pushed)
    {
        m_package.cloud = m_state_data.lidar_buffer.front().second;
        std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(), [](PointType &p1, PointType &p2)
                  { return p1.curvature < p2.curvature; });

        // auto &pts = m_package.cloud->points;
        // if (!pts.empty())
        // {
        //     std::cout << std::fixed << std::setprecision(0)
        //               << "[校验] first curvature: " << pts.front().curvature
        //               << ", last curvature: " << pts.back().curvature
        //               << ", diff: " << (pts.back().curvature - pts.front().curvature)
        //               << std::endl;
        // }
        m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
        m_package.cloud_end_time = m_package.cloud_start_time + double(m_package.cloud->points.back().curvature);
        m_state_data.lidar_pushed = true;
    }
    if (m_state_data.last_imu_time < m_package.cloud_end_time)
        return false;

    Vec<IMUData>().swap(m_package.imus);
    while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
    {
        m_package.imus.emplace_back(m_state_data.imu_buffer.front());
        m_state_data.imu_buffer.pop_front();
    }
    m_state_data.lidar_buffer.pop_front();
    m_state_data.lidar_pushed = false;
    return true;
}

void LIONode::execute()
{
    if (lidar_vec.empty() || imu_vec.empty())
    {
        std::cerr << "Lidar or IMU data is empty, please check the rosbag file." << std::endl;
        return;
    }
    BOOST_FOREACH (const sensor_msgs::ImuPtr &imu_msg, imu_vec)
    {
        imuCB(imu_msg);
    }
    int frame = 0;
    BOOST_FOREACH (const sensor_msgs::PointCloud2Ptr &lidar_msg, lidar_vec)
    {
        std::cout << "Processing frame: " << frame << std::endl;
        lidarCB(lidar_msg);
        while (syncPackage())
        {
            // process the package
            auto t1 = std::chrono::high_resolution_clock::now();
            m_builder->process(m_package);
            auto t2 = std::chrono::high_resolution_clock::now();
            if (m_node_config.print_time_cost)
            {
                auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
                std::cout << "LIO Processing time: " << std::fixed << std::setprecision(2) << time_used << " ms" << std::endl;
            }

            if (m_builder->status() != BuilderStatus::MAPPING)
                break;
            body_cloud_ = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);
            world_cloud_ = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_builder->lidar_processor()->r_wl(), m_builder->lidar_processor()->t_wl());
            r_wl_ = m_builder->lidar_processor()->r_wl();
            t_wl_ = m_builder->lidar_processor()->t_wl();
            if (1)
            {
                std::cout << "r_wl: " << r_wl_.transpose() << std::endl;
                std::cout << "t_wl: " << t_wl_.transpose() << std::endl;
            }
            *global_map += *world_cloud_;
            if (frame % m_builder_config.map_save_interval == 0)
            {
                std::cout << "Saving global map to global_map.pcd" << std::endl;
                pcl::io::savePCDFileASCII("global_map" + std::to_string(frame) + ".pcd", *global_map);
            }
            frame++;
        }
    }
}

double LIONode::scan_end_time()
{
    return m_package.cloud_end_time * 1e-9; // Convert nanoseconds to seconds
}

void LIONode::processPackage()
{
    m_builder->process(m_package);
}

bool LIONode::checkMappingStatus() const
{
    return m_builder->status() != BuilderStatus::MAPPING;
}

CloudType::Ptr LIONode::getLidarCloud()
{
    return m_package.cloud;
}

CloudType::Ptr LIONode::getBodyCloud()
{
    return m_builder->lidar_processor()->transformCloud(m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);
}

CloudType::Ptr LIONode::getWorldCloud()
{
    return m_builder->lidar_processor()->transformCloud(m_package.cloud, m_builder->lidar_processor()->r_wl(), m_builder->lidar_processor()->t_wl());
}

M3D LIONode::getRWL()
{
    return m_builder->lidar_processor()->r_wl();
}
V3D LIONode::getTWL()
{
    return m_builder->lidar_processor()->t_wl();
}