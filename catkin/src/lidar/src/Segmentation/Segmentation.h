#ifndef __SEGMENTATION_H__
#define __SEGMENTATION_H__

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Segment{
public:
    Segment();

    void setDistanceThreshold(int thresh);

    pcl::PointCloud<pcl::PointXYZ> segmentData(pcl::PointCloud<pcl::PointXYZ>::ptr &input_cloud);

private:
    int _distanceThreshold;
}

#endif