/*
 * @Description: Do not Edit
 * @Author: hao.lin (voyah perception)
 * @Date: 2025-06-20 21:58:44
 * @LastEditors: Do not Edit
 * @LastEditTime: 2025-06-24 17:15:53
 */
#ifndef VIEWER_H
#define VIEWER_H

#include <Eigen/Core>
#include <thread>
#include <chrono>
#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

class CloudViewer
{
public:
    CloudViewer()
    {
        viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Turbo Colormap Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        viewer->registerKeyboardCallback(&CloudViewer::keyboardEventOccurred, *this);
        pressed_key = -1;
    }

    template<typename PointT>
    void show_custom(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const std::string& cloud_id, int r, int g, int b) {
        if (!cloud || cloud->empty()) return;
        if (viewer->contains(cloud_id)) {
            viewer->removePointCloud(cloud_id);
        }
        pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler(cloud, r, g, b);
        viewer->addPointCloud<PointT>(cloud, color_handler, cloud_id);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_id);
        pressed_key = -1;
    }

    template <typename PointT>
    void show(const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const std::string &cloud_id = "cloud")
    {
        if (!cloud || cloud->empty())
            return;
        if (viewer->contains(cloud_id))
        {
            viewer->removePointCloud(cloud_id);
        }
        viewer->addPointCloud<PointT>(cloud, cloud_id);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_id);
        pressed_key = -1;
    }
    int spin_once_wait_key()
    {
        pressed_key = -1;
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(10);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            if (pressed_key != -1)
                return pressed_key;
        }
        return 27;
    }

private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    int pressed_key;
    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *)
    {
        if (event.keyDown())
        {
            if (event.getKeySym() == "space")
                pressed_key = ' ';
            if (event.getKeySym() == "a" || event.getKeySym() == "A")
                pressed_key = 'a';
            if (event.getKeySym() == "d" || event.getKeySym() == "D" || event.getKeySym() == "Right")
                pressed_key = 'd';
            if (event.getKeySym() == "Escape")
                pressed_key = 27;
        }
    }
};
#endif // VIEWER_H