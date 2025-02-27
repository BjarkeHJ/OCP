#include <chrono>
#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree_search.h>

class LidarVoxelMapping : public rclcpp::Node {
public:
    LidarVoxelMapping() : Node("lidar_voxel_mapping") {
        pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/isaac/point_cloud_0", 10,
            std::bind(&LidarVoxelMapping::pcdCallback,  this, std::placeholders::_1));
        
        pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/RP_PointCloud", 10);
    }

private:
    // ROS2 stuff...
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;

    // Initialize global voxel map
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    
    // parameters
    float z_limit = 0.1;
    float leaf_size = 0.1;

    void pcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg) {
        // Convert point cloud message to pcl object
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pointcloud_msg, *cloud);
        
        // Slice and remove points below - Crude ground extraction
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sliced(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> slicer;
        slicer.setInputCloud(cloud);
        slicer.setFilterFieldName("z");
        slicer.setFilterLimits(z_limit, std::numeric_limits<float>::max());
        slicer.filter(*cloud_sliced);

        // Downsample cloud using voxelfilter 
        pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
        voxelGrid.setInputCloud(cloud_sliced);
        voxelGrid.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxelGrid.filter(*filteredCloud);

        // Convert back to ROS2 msg for republishing (Visualization)
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*filteredCloud, output);
        output.header.frame_id = "lidar_frame";
        output.header.stamp = this->get_clock()->now();
        pcd_pub_->publish(output);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::Logger logger = rclcpp::get_logger("lidar_voxel_mapping");
    RCLCPP_INFO_ONCE(logger, "Starting Path Planning Node...");
    rclcpp::spin(std::make_shared<LidarVoxelMapping>());
    rclcpp::shutdown();
    return 0;
}
