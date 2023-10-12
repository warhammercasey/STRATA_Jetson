#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class Visualize
{
public:
    Visualize();

    // Set the input point cloud to visualize
    void setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud);

    // Set the detected objects to visualize
    void setDetectedObjects(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &objects);

    // Visualize the point cloud and detected objects
    void visualizePointCloud();

private:
    ros::NodeHandle nh_;
    pcl::visualization::PCLVisualizer viewer_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> detected_objects_;
};

#endif // VISUALIZE_H
