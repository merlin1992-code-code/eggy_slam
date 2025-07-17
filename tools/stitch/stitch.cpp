/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-07-06 22:35:42
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-17 09:25:35
 */
#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <limits>

using CloudType = pcl::PointCloud<pcl::PointXYZINormal>;
using json = nlohmann::json;

void project_and_save(const CloudType::Ptr &cloud, float res, const std::string &prefix)
{
    float min_x = std::numeric_limits<float>::max(), max_x = std::numeric_limits<float>::lowest();
    float min_y = min_x, max_y = max_x, min_z = min_x, max_z = max_x;
    for (const auto &pt : cloud->points)
    {
        min_x = std::min(min_x, pt.x);
        max_x = std::max(max_x, pt.x);
        min_y = std::min(min_y, pt.y);
        max_y = std::max(max_y, pt.y);
        min_z = std::min(min_z, pt.z);
        max_z = std::max(max_z, pt.z);
    }
    int w_xy = static_cast<int>((max_x - min_x) / res) + 1;
    int h_xy = static_cast<int>((max_y - min_y) / res) + 1;
    int w_xz = w_xy, h_xz = static_cast<int>((max_z - min_z) / res) + 1;
    int w_yz = static_cast<int>((max_y - min_y) / res) + 1, h_yz = h_xz;

    cv::Mat sum_xy = cv::Mat::zeros(h_xy, w_xy, CV_32FC1), cnt_xy = cv::Mat::zeros(h_xy, w_xy, CV_32FC1);
    cv::Mat sum_xz = cv::Mat::zeros(h_xz, w_xz, CV_32FC1), cnt_xz = cv::Mat::zeros(h_xz, w_xz, CV_32FC1);
    cv::Mat sum_yz = cv::Mat::zeros(h_yz, w_yz, CV_32FC1), cnt_yz = cv::Mat::zeros(h_yz, w_yz, CV_32FC1);

    for (const auto &pt : cloud->points)
    {
        int ix = static_cast<int>((pt.x - min_x) / res);
        int iy = static_cast<int>((pt.y - min_y) / res);
        int iz = static_cast<int>((pt.z - min_z) / res);
        // xy
        if (ix >= 0 && ix < w_xy && iy >= 0 && iy < h_xy)
        {
            sum_xy.at<float>(h_xy - 1 - iy, ix) += pt.intensity;
            cnt_xy.at<float>(h_xy - 1 - iy, ix) += 1.0f;
        }
        // xz
        if (ix >= 0 && ix < w_xz && iz >= 0 && iz < h_xz)
        {
            sum_xz.at<float>(h_xz - 1 - iz, ix) += pt.intensity;
            cnt_xz.at<float>(h_xz - 1 - iz, ix) += 1.0f;
        }
        // yz
        if (iy >= 0 && iy < w_yz && iz >= 0 && iz < h_yz)
        {
            sum_yz.at<float>(h_yz - 1 - iz, iy) += pt.intensity;
            cnt_yz.at<float>(h_yz - 1 - iz, iy) += 1.0f;
        }
    }

    auto make_color_turbo = [](const cv::Mat &sum, const cv::Mat &cnt)
    {
        cv::Mat avg, norm, color;
        cv::divide(sum, cnt, avg, 1, CV_32FC1);
        double minVal, maxVal;
        cv::minMaxLoc(avg, &minVal, &maxVal, nullptr, nullptr, cnt > 0);
        // 归一化到0~255
        avg.setTo(0, cnt == 0);
        avg.convertTo(norm, CV_8UC1, 255.0 / (maxVal - minVal + 1e-6), -minVal * 255.0 / (maxVal - minVal + 1e-6));
        cv::applyColorMap(norm, color, cv::COLORMAP_TURBO);
        color.setTo(cv::Scalar(0, 0, 0), cnt == 0); // 无点处设为黑色
        return color;
    };

    auto make_color_turbo_log = [](const cv::Mat &sum, const cv::Mat &cnt)
    {
        cv::Mat avg, log_img, norm, color;
        cv::divide(sum, cnt, avg, 1, CV_32FC1);
        avg.setTo(0, cnt == 0);

        // 对数拉伸
        cv::log(avg + 1, log_img);

        // 计算分位数
        std::vector<float> vals;
        for (int r = 0; r < log_img.rows; ++r)
            for (int c = 0; c < log_img.cols; ++c)
                if (cnt.at<float>(r, c) > 0)
                    vals.push_back(log_img.at<float>(r, c));
        std::sort(vals.begin(), vals.end());
        float minVal = vals[vals.size() * 0.01];
        float maxVal = vals[vals.size() * 0.99];

        log_img.convertTo(norm, CV_8UC1, 255.0 / (maxVal - minVal + 1e-6), -minVal * 255.0 / (maxVal - minVal + 1e-6));
        cv::applyColorMap(norm, color, cv::COLORMAP_TURBO);
        color.setTo(cv::Scalar(0, 0, 0), cnt == 0);
        return color;
    };

    auto make_color_binary = [](const cv::Mat &sum, const cv::Mat &cnt)
    {
        cv::Mat avg, norm, binary;
        cv::divide(sum, cnt, avg, 1, CV_32FC1);
        avg.setTo(0, cnt == 0);

        // 计算有效像素的中位数作为阈值
        std::vector<float> vals;
        for (int r = 0; r < avg.rows; ++r)
            for (int c = 0; c < avg.cols; ++c)
                if (cnt.at<float>(r, c) > 0)
                    vals.push_back(avg.at<float>(r, c));
        if (vals.empty())
            return cv::Mat();
        std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
        float thresh = vals[vals.size() / 2];

        // 归一化到0~255
        double minVal, maxVal;
        cv::minMaxLoc(avg, &minVal, &maxVal, nullptr, nullptr, cnt > 0);
        avg.convertTo(norm, CV_8UC1, 255.0 / (maxVal - minVal + 1e-6), -minVal * 255.0 / (maxVal - minVal + 1e-6));

        // 二值化
        cv::threshold(norm, binary, 255.0 * (thresh - minVal) / (maxVal - minVal + 1e-6), 255, cv::THRESH_BINARY);
        binary.setTo(0, cnt == 0);                        // 无点处设为黑色
        cv::cvtColor(binary, binary, cv::COLOR_GRAY2BGR); // 转为3通道
        return binary;
    };

    cv::imwrite(prefix + "_xy_turbo.png", make_color_turbo_log(sum_xy, cnt_xy));
    cv::imwrite(prefix + "_xz_turbo.png", make_color_turbo_log(sum_xz, cnt_xz));
    cv::imwrite(prefix + "_yz_turbo.png", make_color_turbo_log(sum_yz, cnt_yz));

    cv::imwrite(prefix + "_xy_binary.png", make_color_binary(sum_xy, cnt_xy));
    cv::imwrite(prefix + "_xz_binary.png", make_color_binary(sum_xz, cnt_xz));
    cv::imwrite(prefix + "_yz_binary.png", make_color_binary(sum_yz, cnt_yz));
}

bool ends_with(const std::string &str, const std::string &suffix)
{
    return str.size() >= suffix.size() &&
           str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// 读取 YAML 格式的 pose
void stitch_from_yaml(const std::string &pose_yaml, const std::string &pcd_dir, CloudType::Ptr global_map)
{
    YAML::Node poses = YAML::LoadFile(pose_yaml);
    for (auto it = poses.begin(); it != poses.end(); ++it)
    {
        std::string file_name = it->first.as<std::string>();
        auto node = it->second;
        auto pos = node["pos"];
        auto rot = node["rot"];
        Eigen::Vector3d t(pos[0].as<double>(), pos[1].as<double>(), pos[2].as<double>());
        Eigen::Quaterniond q(rot[3].as<double>(), rot[0].as<double>(), rot[1].as<double>(), rot[2].as<double>());

        CloudType::Ptr cloud(new CloudType);
        std::string pcd_path = pcd_dir + "/" + file_name;
        if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(pcd_path, *cloud) == -1)
        {
            std::cerr << "Failed to load " << pcd_path << std::endl;
            continue;
        }
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T.block<3, 1>(0, 3) = t;
        CloudType::Ptr cloud_world(new CloudType);
        pcl::transformPointCloud(*cloud, *cloud_world, T.cast<float>());
        *global_map += *cloud_world;
    }
}

// 读取 JSON 格式的 pose
void stitch_from_json(const std::string &pose_json, const std::string &pcd_dir, CloudType::Ptr global_map)
{
    std::ifstream fin(pose_json);
    if (!fin.is_open())
    {
        std::cerr << "Failed to open " << pose_json << std::endl;
        return;
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

        CloudType::Ptr cloud(new CloudType);
        std::string pcd_path = pcd_dir + "/" + file_name;
        if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(pcd_path, *cloud) == -1)
        {
            std::cerr << "Failed to load " << pcd_path << std::endl;
            continue;
        }
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T.block<3, 1>(0, 3) = t;
        CloudType::Ptr cloud_world(new CloudType);
        pcl::transformPointCloud(*cloud, *cloud_world, T.cast<float>());
        *global_map += *cloud_world;
    }
}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " poses.yaml|poses.json pcd_dir [output.pcd]" << std::endl;
        return 1;
    }
    std::string pose_file = argv[1];
    std::string pcd_dir = argv[2];
    std::string output_pcd = argc > 3 ? argv[3] : "global_map.pcd";

    CloudType::Ptr global_map(new CloudType);

    if (ends_with(pose_file, ".yaml") || ends_with(pose_file, ".yml"))
    {
        stitch_from_yaml(pose_file, pcd_dir, global_map);
    }
    else if (ends_with(pose_file, ".json"))
    {
        stitch_from_json(pose_file, pcd_dir, global_map);
    }
    else
    {
        std::cerr << "Unsupported pose file format: " << pose_file << std::endl;
        return 1;
    }

    pcl::io::savePCDFileBinary(output_pcd, *global_map);
    std::cout << "Saved global map: " << output_pcd << std::endl;
    project_and_save(global_map, 0.01f, "global_map");
    return 0;
}