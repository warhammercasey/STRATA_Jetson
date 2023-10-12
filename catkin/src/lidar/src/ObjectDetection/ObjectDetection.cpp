#include "ObjectDetection.h"
#include <pcl/console/time.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

ObjectDetection::ObjectDetection() : nh_("~"), minObjectSize(0.10)
{
    // Create a publisher for publishing object locations
    object_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("object_locations", 1);
}

void ObjectDetection::detectObjects(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud)
{
    // Object detection logic

    // Create a EuclideanClusterExtraction object
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    // Set the cluster tolerance (maximum distance between points in a cluster)
    ec.setClusterTolerance(0.05); // 5 cm (adjust as needed)

    // Set the minimum cluster size (minimum number of points in a cluster)
    ec.setMinClusterSize(10); // 10 points (adjust as needed)

    // Set the input point cloud
    ec.setInputCloud(input_cloud);

    // Create a vector to store cluster indices
    std::vector<pcl::PointIndices> cluster_indices;

    // Perform clustering
    ec.extract(cluster_indices);

    // Clear the existing detected objects
    detected_objects_.clear();

    // Extract detected objects based on cluster indices
    for (const auto &indices : cluster_indices)
    {
        if (indices.indices.size() >= minObjectSize)
        {
            // Create an object only if it meets the minimum size requirement
            pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto &idx : indices.indices)
            {
                object->points.push_back(input_cloud->points[idx]);
            }
            object->width = object->points.size();
            object->height = 1;
            object->is_dense = true;

            detected_objects_.push_back(object);
        }
    }

    // Publish the locations of detected objects
    publishObjectLocations(detected_objects_, input_cloud->header);
}

void ObjectDetection::setMinObjectSizeInCM(float minSizeCM)
{
    // Convert the minimum object size from centimeters to meters
    minObjectSize = minSizeCM / 100.0; // 1 cm = 0.01 meters
}

const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &ObjectDetection::getDetectedObjects() const
{
    return detected_objects_;
}

// Function to publish object locations
void ObjectDetection::publishObjectLocations(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &objects, const pcl::PCLHeader &header)
{
    // Create a point cloud message for object locations
    pcl::PointCloud<pcl::PointXYZ> object_locations;
    for (const auto &object : objects)
    {
        for (const auto &point : object->points)
        {
            // Add the object point to the location point cloud
            object_locations.points.push_back(point);
        }
    }

    // Publish the object locations
    sensor_msgs::PointCloud2 object_cloud;
    pcl::toROSMsg(object_locations, object_cloud);

    // Manually copy the header fields
    object_cloud.header.seq = header.seq;
    object_cloud.header.stamp = ros::Time::now(); // You can set the timestamp as needed
    object_cloud.header.frame_id = header.frame_id;

    object_pub_.publish(object_cloud);
}
