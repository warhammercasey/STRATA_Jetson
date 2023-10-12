// Visualize.cpp

#include "Visualize.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

Visualize::Visualize() : nh_("~")
{
    // Initialize the PCL visualizer
    viewer_ = pcl::visualization::PCLVisualizer("Point Cloud Viewer");

    // Set the background color (optional)
    viewer_.setBackgroundColor(0, 0, 0);

    // Create a coordinate system axes (optional)
    viewer_.addCoordinateSystem(1.0);
}

void Visualize::setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud)
{
    input_cloud_ = input_cloud;
}

void Visualize::setDetectedObjects(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &objects)
{
    detected_objects_ = objects;
}

void Visualize::visualizePointCloud()
{
    // Visualize the input point cloud in green
    viewer_.removePointCloud("input_cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_color(input_cloud_, 0, 255, 0); // Green
    viewer_.addPointCloud<pcl::PointXYZ>(input_cloud_, input_color, "input_cloud");

    // Visualize detected objects in red
    for (size_t i = 0; i < detected_objects_.size(); ++i)
    {
        std::string object_name = "object_" + std::to_string(i);
        viewer_.removePointCloud(object_name);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> object_color(detected_objects_[i], 255, 0, 0); // Red
        viewer_.addPointCloud<pcl::PointXYZ>(detected_objects_[i], object_color, object_name);
    }

    // Spin the visualizer to display the point cloud
    while (!viewer_.wasStopped())
    {
        viewer_.spinOnce(100); // Allow interactive updates
    }
}
