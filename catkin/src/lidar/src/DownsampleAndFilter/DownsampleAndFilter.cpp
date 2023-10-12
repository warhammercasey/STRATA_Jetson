#include "DownsampleAndFilter.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

// Constructor for the DownsampleAndFilter class
// Initializes the default voxel grid leaf size to 0.01 meters
DownsampleAndFilter::DownsampleAndFilter() : leaf_size_(0.01)
{
    // Default voxel grid leaf size
}

// Set the voxel grid leaf size
void DownsampleAndFilter::setLeafSize(float leaf_size)
{
    leaf_size_ = leaf_size;
}

// Downsampling function using voxel grid filtering
// Takes an input point cloud and downsamples it
void DownsampleAndFilter::downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud)
{
    // Create a VoxelGrid filter object
    pcl::VoxelGrid<pcl::PointXYZ> sor;

    // Set the input cloud for the filter
    sor.setInputCloud(input_cloud);

    // Set the leaf size for the VoxelGrid filter
    sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);

    // Apply the VoxelGrid filter to downsample the input cloud
    sor.filter(*output_cloud);
}

// Function to remove outliers using StatisticalOutlierRemoval filter
void DownsampleAndFilter::removeOutliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                                         int mean_k, double stddev_mul_thresh)
{
    // Create a StatisticalOutlierRemoval filter object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

    // Set the input cloud for the filter
    sor.setInputCloud(input_cloud);

    // Set the number of nearest neighbors to consider for mean distance estimation
    sor.setMeanK(mean_k);

    // Set the standard deviation multiplier threshold
    sor.setStddevMulThresh(stddev_mul_thresh);

    // Apply the StatisticalOutlierRemoval filter to remove outliers
    sor.filter(*output_cloud);
}
