// ObjectDetection.h

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <vector>

class ObjectDetection
{
public:
    ObjectDetection();

    // Other class functions...

    // Setter to configure the minimum object size in centimeters
    void setMinObjectSizeInCM(float minSizeCM);

    // Detect objects in the input point cloud
    void detectObjects(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud);

    // Getter to retrieve the detected objects
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &getDetectedObjects() const;

private:
    ros::NodeHandle nh_;
    ros::Publisher object_pub_;
    float minObjectSize; // Minimum object size in meters

    // Vector to store detected objects
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> detected_objects_;

    // Function to publish object locations
    void publishObjectLocations(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &objects, const pcl::PCLHeader &header);
};