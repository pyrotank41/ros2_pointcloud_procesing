#ifndef POINTCLOUDPROCESSOR_HPP
#define POINTCLOUDPROCESSOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>

class PointCloudProcessor {
public:
    void process(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void removeOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double meanK = 50, double stdDevMulThresh = 1.0);
    void removeOutliersBruteForce(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float searchRadius, int minNeighbors);
    void removeOutliersUsingKDTree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float searchRadius, int minNeighbors);
    void voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float voxelSize);
};

#endif // POINTCLOUDPROCESSOR_HPP
