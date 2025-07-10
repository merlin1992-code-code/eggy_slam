#ifndef DYN_OBJ_CLUS_H
#define DYN_OBJ_CLUS_H

#include <iostream>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <unordered_map>
#include <unordered_set>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <cluster_predict/DBSCAN_kdtree.h>
#include <cluster_predict/EA_disk.h>
#include <cluster_predict/voxel_cluster.h>

namespace cluster
{

  typedef pcl::PointXYZINormal PointType;
  typedef std::vector<pcl::PointCloud<PointType>> VoxelMap;
#define HASH_length 10000

  namespace std_msgs
  {
    struct Header
    {
      uint32_t seq;
      int64_t stamp; // nanoseconds since epoch
      std::string frame_id;
    };
  } // namespace std_msgs

  namespace geometry_msgs
  {
    struct Header
    {
      uint32_t seq;
      int64_t stamp; // nanoseconds since epoch
      std::string frame_id;
    };

    struct Point
    {
      double x;
      double y;
      double z;
    };

    struct Quaternion
    {
      double x;
      double y;
      double z;
      double w;
    };

    struct Pose
    {
      Point position;
      Quaternion orientation;
    };

    struct PoseWithCovariance
    {
      Pose pose;
      double covariance[36]; // 6x6 matrix in row-major order
    };

    struct PoseWithCovarianceStamped
    {
      Header header;
      PoseWithCovariance pose;
    };
  } // namespace geometry_msgs

  namespace ros
  {
    struct Time
    {
      uint32_t sec;
      uint32_t nsec;

      Time() : sec(0), nsec(0) {}
      Time(uint32_t s, uint32_t ns) : sec(s), nsec(ns) {}

      // 添加 fromSec 静态方法
      static Time fromSec(double t)
      {
        uint32_t sec = static_cast<uint32_t>(t);
        uint32_t nsec = static_cast<uint32_t>((t - sec) * 1e9);
        // 处理纳秒溢出（如 1.999999999 -> 2秒 -1纳秒）
        if (nsec >= 1e9)
        {
          sec += 1;
          nsec -= 1e9;
        }
        return Time(sec, nsec);
      }

      static Time now()
      {
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto sec = std::chrono::duration_cast<std::chrono::seconds>(duration);
        auto nsec =
            std::chrono::duration_cast<std::chrono::nanoseconds>(duration - sec);
        return Time(sec.count(), nsec.count());
      }

      struct Duration
      {
        double sec;
        explicit operator double() const { return sec; }
        double toSec() const { return sec; }
      };

      Duration operator-(const Time &other) const
      {
        int64_t delta_sec = sec - other.sec;
        int64_t delta_nsec = nsec - other.nsec;
        if (delta_nsec < 0)
        {
          delta_sec -= 1;
          delta_nsec += 1e9;
        }
        return Duration{delta_sec + delta_nsec / 1e9};
      }

      int64_t toMicro() const
      {
        return static_cast<int64_t>(sec) * 1000000 + nsec / 1000;
      }

      int64_t toMilli() const
      {
        return static_cast<int64_t>(sec) * 1000 + nsec / 1000000;
      }
    };
  } // namespace ros

  struct bbox_t
  {
    std::vector<std::vector<int>> Point_indices;
    std::vector<pcl::PointCloud<PointType>> Point_cloud;
    pcl::PointCloud<PointType>::Ptr Points;
    std::vector<geometry_msgs::PoseWithCovarianceStamped> Center;
    std::vector<pcl::PointCloud<PointType>> Ground_points;
    std::vector<pcl::PointCloud<PointType>> true_ground;
    std::vector<std::unordered_set<int>> Ground_voxels_set;
    std::vector<std::vector<int>> Ground_voxels_vec;
    std::vector<Eigen::Vector3f> OOBB_Min_point;
    std::vector<Eigen::Vector3f> OOBB_Max_point;
    std::vector<Eigen::Matrix3f> OOBB_R;
    std::vector<int> umap_points_num;

    void reset()
    {
      Point_indices.clear();
      Point_cloud.clear();
      Center.clear();
      Ground_points.clear();
      true_ground.clear();
      Ground_voxels_set.clear();
      Ground_voxels_vec.clear();
      OOBB_Min_point.clear();
      OOBB_Max_point.clear();
      OOBB_R.clear();
    }
  };

  class DynObjCluster
  {
  public:
    int nn_points_size = 3;
    float nn_points_radius = 0.6f;
    int min_cluster_size = 8;
    int max_cluster_size = 25000;
    int cluster_extend_pixel = 2;
    int cluster_min_pixel_number = 4;
    float Voxel_revolusion = 0.3f;
    int time_ind = 0;
    double time_total = 0.0, time_total_average = 0.0;
    ros::Time cluster_begin;
    int cur_frame = 0;
    float thrustable_thresold = 0.3f;
    std::string out_file = "";
    std::ofstream out;
    bool debug_en{false};
    std_msgs::Header header;
    Eigen::Matrix3d odom_rot;
    Eigen::Vector3d odom_pos;
    std::vector<Point_Cloud> umap;
    std::vector<Point_Cloud> umap_ground;
    std::vector<Point_Cloud> umap_insidebox;
    int GridMapsize;
    int GridMapedgesize_xy;
    int GridMapedgesize_z;
    Eigen::Vector3f xyz_origin;
    Eigen::Vector3f maprange;
    // ros::Publisher pub_pcl_dyn_extend;

    DynObjCluster() {};
    ~DynObjCluster() {};

    // void Init(ros::Publisher &pub_pcl_dyn_extend_in, ros::Publisher
    // &cluster_vis_high_in, ros::Publisher &pub_ground_points_in);
    void Init();
    void Clusterprocess(std::vector<int> &dyn_tag,
                        pcl::PointCloud<PointType> event_point,
                        const pcl::PointCloud<PointType> &raw_point,
                        const std_msgs::Header &header_in,
                        const Eigen::Matrix3d odom_rot_in,
                        const Eigen::Vector3d odom_pos_in,
                        pcl::PointCloud<PointType> &true_ground_out);
    void ClusterAndTrack(std::vector<int> &dyn_tag,
                         pcl::PointCloud<PointType>::Ptr &points_in,
                         std_msgs::Header header_in, bbox_t &bbox, double delta,
                         const pcl::PointCloud<PointType> &raw_point);
    void GetClusterResult(pcl::PointCloud<PointType>::Ptr points_in,
                          std::vector<pcl::PointIndices> &cluster_indices);
    void GetClusterResult_voxel(pcl::PointCloud<PointType>::Ptr points_in,
                                std::vector<Point_Cloud> &umap_in,
                                std::vector<std::vector<int>> &voxel_clusters,
                                std::unordered_set<int> &used_map_set);
    void PubClusterResult_voxel(std::vector<int> &dyn_tag,
                                std_msgs::Header current_header, bbox_t &bbox,
                                double delta,
                                std::vector<std::vector<int>> &voxel_clusters,
                                const pcl::PointCloud<PointType> &raw_point,
                                std::unordered_set<int> &used_map_set);
    bool ground_estimate(const pcl::PointCloud<PointType> &ground_pcl,
                         const Eigen::Vector3f &world_z,
                         Eigen::Vector3f &ground_norm,
                         Eigen::Vector4f &ground_plane,
                         pcl::PointCloud<PointType> &true_ground,
                         std::unordered_set<int> &extend_pixels);
    void ground_remove(const Eigen::Vector4f &ground_plane,
                       pcl::PointCloud<PointType> &cluster_pcl,
                       std::vector<int> &cluster_pcl_ind,
                       std::vector<int> &dyn_tag,
                       pcl::PointCloud<PointType> &true_ground,
                       std::vector<Point_Cloud> &umap);
    void isolate_remove(pcl::PointCloud<PointType> &cluster_pcl,
                        std::vector<int> &cluster_pcl_ind,
                        std::vector<int> &dyn_tag);
    void oobb_estimate(const VoxelMap &vmap,
                       const pcl::PointCloud<PointType> &points,
                       Eigen::Vector3f &min_point_obj,
                       Eigen::Vector3f &max_point_obj, Eigen::Matrix3f &R,
                       const Eigen::Vector3f ground_norm);
    void event_extend(const Eigen::Matrix3f &R, bool ground_detect, bbox_t &bbox,
                      std::vector<int> &dyn_tag, const int &bbox_index);
    bool esti_plane(Eigen::Vector4f &pca_result,
                    const pcl::PointCloud<PointType> &point);
    void XYZExtract(const int &position, Eigen::Vector3f &xyz);
  };
} // namespace cluster

#endif
