#ifndef DOWNSAMPLEANDFILTER_H
#define DOWNSAMPLEANDFILTER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class DownsampleAndFilter
{
public:
    DownsampleAndFilter();

    void setLeafSize(float leaf_size);

    void downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud);

    // Added: Function declaration to remove outliers
    void removeOutliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                        int mean_k, double stddev_mul_thresh);

private:
    float leaf_size_;
};

#endif // DOWNSAMPLEANDFILTER_H
