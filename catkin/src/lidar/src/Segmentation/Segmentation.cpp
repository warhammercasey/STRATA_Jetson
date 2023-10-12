#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include "Segmentation.h"

Segment::Segment(){

}

void Segment::setDistanceThreshold(int thresh){
    if(thresh <= 0) return;

    _distanceThreshold = thresh;
}

pcl::PointCloud<pcl::PointXYZ> Segment::segmentData(pcl::PointCloud<pcl::PointXYZ>::ptr &input_cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (_distanceThreshold);

    int nr_points = (int) input_cloud->size();
    while(input_cloud->size() > 0.3 * nr_points){
        seg.setInputCloud(input_cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size () == 0){
            ROS_WARN("Could not estimate a planar model for the given dataset.");
            break;
        }

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (input_cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);

        extract.filter (*cloud_plane);

        extract.setNegative (true);
        extract.filter (*cloud_f);
        *input_cloud = *cloud_f;
    }

    
}