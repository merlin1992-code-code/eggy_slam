/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-06-24 09:44:02
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-06-27 16:03:15
 */
#include "utils.h"

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::convertToPCL(const sensor_msgs::PointCloud2Ptr pc_msg, int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    int point_num = pc_msg->height * pc_msg->width;
    cloud->reserve(point_num / filter_num + 1);

    double ts_begin = pc_msg->header.stamp.toNSec();
    int64_t ts_begin_ns = static_cast<int64_t>(ts_begin);

    int x_idx = getPointCloud2FieldIndex(*pc_msg, "x");
    int y_idx = getPointCloud2FieldIndex(*pc_msg, "y");
    int z_idx = getPointCloud2FieldIndex(*pc_msg, "z");
    int i_idx = getPointCloud2FieldIndex(*pc_msg, "intensity");
    int ts_idx = getPointCloud2FieldIndex(*pc_msg, "timestamp");

    int x_offset = pc_msg->fields[x_idx].offset;
    int y_offset = pc_msg->fields[y_idx].offset;
    int z_offset = pc_msg->fields[z_idx].offset;
    int i_offset = pc_msg->fields[i_idx].offset;
    int ts_offset = pc_msg->fields[ts_idx].offset;

    int step = pc_msg->point_step;
    int num_invalid_pt = 0;
    int num_out_of_range_pt = 0;
    for (size_t j = 0; j < point_num; ++j)
    {
        float x = sensor_msgs::readPointCloud2BufferValue<float>(
            &pc_msg->data[j * step + x_offset],
            pc_msg->fields[x_idx].datatype);
        float y = sensor_msgs::readPointCloud2BufferValue<float>(
            &pc_msg->data[j * step + y_offset],
            pc_msg->fields[y_idx].datatype);
        float z = sensor_msgs::readPointCloud2BufferValue<float>(
            &pc_msg->data[j * step + z_offset],
            pc_msg->fields[z_idx].datatype);
        float intensity = sensor_msgs::readPointCloud2BufferValue<float>(
            &pc_msg->data[j * step + i_offset],
            pc_msg->fields[i_idx].datatype);
        double ts = sensor_msgs::readPointCloud2BufferValue<double>(
            &pc_msg->data[j * step + ts_offset],
            pc_msg->fields[ts_idx].datatype);
        // 过滤无效点

        if (std::isnan(x) || std::isnan(y) || std::isnan(z))
        {
            num_invalid_pt += 1;
            continue;
        }

        // 过滤范围外的点
        float r2 = x * x + y * y + z * z;
        if (r2 < min_range * min_range || r2 > max_range * max_range)
        {
            num_out_of_range_pt += 1;
            continue;
        }


        int64_t ts_ns = static_cast<int64_t>(ts * 1e9);
        int64_t offset_time_ns = ts_ns - ts_begin_ns;

        pcl::PointXYZINormal p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = intensity;
        p.curvature = static_cast<float>(offset_time_ns);

        if (p.curvature < 0 || p.curvature >= 1e9)
        {
            std::cout << std::fixed << std::setprecision(9)
                      << "[Warning] Abnormal curvature: ts=" << ts
                      << " curvature=" << p.curvature << std::endl;
        }
        cloud->push_back(p);
    }
    std::cout << "Converted PointCloud2 to PCL cloud with " << cloud->size() << " points, "
              << num_invalid_pt << " invalid points filtered. "
              << num_out_of_range_pt << " out of range points filtered. " << std::endl;

    return cloud;
}

double Utils::getSec(std_msgs::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nsec) * 1e-9;
}

void Utils::ParseRosbagToVectors(
    const std::string &bag_path,
    const std::string &lidar_topic,
    const std::string &imu_topic,
    std::vector<sensor_msgs::PointCloud2Ptr> &lidar_vec,
    std::vector<sensor_msgs::ImuPtr> &imu_vec)
{
    std::cout << "bag_path: " << bag_path << std::endl;
    rosbag::Bag bag;
    try
    {
        bag.open(bag_path, rosbag::bagmode::Read);
    }
    catch (const rosbag::BagException &e)
    {
        std::cerr << "Failed to open bag: " << e.what() << std::endl;
        return;
    }
    std::vector<std::string> topics{lidar_topic, imu_topic};
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    size_t lidar_count = 0, imu_count = 0, total = 0;

    BOOST_FOREACH (rosbag::MessageInstance const &m, view)
    {
        ++total;
        if (m.getTopic() == lidar_topic || ("/" + m.getTopic() == lidar_topic))
        {
            sensor_msgs::PointCloud2Ptr pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (pc_msg)
            {
                lidar_vec.push_back(pc_msg);
                ++lidar_count;
            }
        }
        else if (m.getTopic() == imu_topic || ("/" + m.getTopic() == imu_topic))
        {
            sensor_msgs::ImuPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
            if (imu_msg)
            {
                imu_vec.push_back(imu_msg);
                ++imu_count;
            }
        }
    }
    bag.close();

    std::cout << "Total messages in view: " << total << std::endl;
    std::cout << "Lidar messages read: " << lidar_count << " lidar_vec size: " << lidar_vec.size() << std::endl;
    std::cout << "IMU messages read: " << imu_count << " imu_vec size: " << imu_vec.size() << std::endl;
    if (lidar_count == 0)
    {
        std::cerr << "Warning: No lidar messages found for topic: " << lidar_topic << std::endl;
    }
    if (imu_count == 0)
    {
        std::cerr << "Warning: No IMU messages found for topic: " << imu_topic << std::endl;
    }
}