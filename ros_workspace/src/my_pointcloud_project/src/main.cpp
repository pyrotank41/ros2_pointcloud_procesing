#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../scans.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file scans.pcd \n");
        return -1;
    }

    // Voxelization
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.5, 0.5, 0.5); // Example voxel size, adjust as needed
    sor.filter(*cloud_filtered);

    // Save the original cloud
    pcl::io::savePCDFileASCII("../output/original_cloud.pcd", *cloud);
    std::cout << "Saved " << cloud->size() << " data points to original_cloud.pcd." << std::endl;

    // Save the filtered cloud
    pcl::io::savePCDFileASCII("../output/filtered_cloud.pcd", *cloud_filtered);
    std::cout << "Saved " << cloud_filtered->size() << " data points to filtered_cloud.pcd." << std::endl;

    return 0;
}
