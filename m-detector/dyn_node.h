/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-07-01 10:00:43
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-07-07 10:19:05
 */
#include <iostream>
#include <dirent.h>

#include <proj.h>
#include "m_utils.h"
#include "DynObjFilter.h"

class DynNode
{
public:
    DynNode(const std::string &config_path);
    void execute(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, M3D &cur_rot, V3D &cur_pos, double scan_end_time);
    void saveTrajectory(const std::string &out_dir);
    std::string getStdPcdDir();

private:
    YAML::Node node_;
    std::string m_cfg_, root_dir_, output_dir_, output_fusion_dir_;
    std::shared_ptr<NodeHandle> nh_;
    std::shared_ptr<DynObjFilter> DynObjFilt_;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cur_pc;
    M3D cur_rot;
    V3D cur_pos;
    double cur_time;
    std::string file_name;
    std::vector<PoseRecord> pose_records_;
};