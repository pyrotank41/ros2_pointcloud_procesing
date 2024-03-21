#include "PointCloudProcessor.hpp"
#include <iostream>
#include <pcl/filters/statistical_outlier_removal.h>
#include <cmath>

double euclideanDistance(const pcl::PointXYZ &point1, const pcl::PointXYZ &point2) {
    return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2) + pow(point1.z - point2.z, 2));
}

void PointCloudProcessor::process(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // Placeholder for processing the point cloud
    std::cout << "Processing point cloud with " << cloud->points.size() << " points." << std::endl;
    // removeOutliersBruteForce(cloud, 0.1, 10);
    // removeOutliersUsingKDTree(cloud, 1, 200);
    // removeOutliers(cloud);
    voxelGridFilter(cloud, 0.5);
}

// Example function to voxel grid filter a point cloud
void PointCloudProcessor::voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float voxelSize) {
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(voxelSize, voxelSize, voxelSize);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    filter.filter(*filteredCloud);
    *cloud = *filteredCloud;
    std::cout << "Original number of points: " << cloud->size() << " After voxelization: " << filteredCloud->size() << std::endl;
}

void PointCloudProcessor::removeOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double meanK, double stdDevMulThresh) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stdDevMulThresh);
    sor.filter(*cloud);
    std::cout << "Removed " << cloud->points.size() << " outliers." << std::endl;
}

// psudo code for removeOutliersBruteForce
// for each point in pointCloud:
//     neighbors = findNeighborsWithinRadius(point, fixedRadius)
//     if size(neighbors) < minNeighborsThreshold:
//         mark point as outlier
// remove all marked outliers from pointCloud


void PointCloudProcessor::removeOutliersBruteForce(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float searchRadius, int minNeighbors) {
    std::vector<int> toKeep;

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        int neighborCount = 0;
        for (size_t j = 0; j < cloud->points.size(); ++j) {
            if (i != j && euclideanDistance(cloud->points[i], cloud->points[j]) <= searchRadius) {
                ++neighborCount;
            }
        }
        if (neighborCount >= minNeighbors) {
            toKeep.push_back(i);
        }
        std::cout << "Point " << i << " has " << neighborCount << " neighbors." << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    for (int index : toKeep) {
        filtered->push_back(cloud->points[index]);
    }

    *cloud = *filtered;
    std::cout << "Removed " << cloud->points.size() << " outliers." << std::endl;
}

void PointCloudProcessor::removeOutliersUsingKDTree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float searchRadius, int minNeighbors) {
    // Create a KD-tree for the input point cloud
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud);

    // Prepare the output point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // For each point, use the KD-tree to find points within the search radius
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        // Clear previous search results
        pointIdxRadiusSearch.clear();
        pointRadiusSquaredDistance.clear();

        // If there are enough neighbors within the search radius, keep the point
        if (tree->radiusSearch(cloud->points[i], searchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >= minNeighbors) {
            filteredCloud->points.push_back(cloud->points[i]);
        }
        // std::cout << "Point " << i << " has " << pointIdxRadiusSearch.size() << " neighbors." << std::endl;
    }

    // Copy the filtered cloud back to the input cloud
    *cloud = *filteredCloud;
}
