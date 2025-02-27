#include <chrono>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree_search.h>

#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class PathPlanner : public rclcpp::Node {
public:
    PathPlanner() : Node("path_planner") {
        /* ROS2 Subscribers */
        pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/isaac/point_cloud_0", 10,
            std::bind(&PathPlanner::pcdCallback,  this, std::placeholders::_1));
        
        /* ROS2 Publishers */
        pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/RP_PointCloud", 10);
        target_pub_ = this->create_publisher<TrajectorySetpoint>("/target_setpoint", 10);

        /* ROS2 Timers */
        target_pub_timer_ = this->create_wall_timer(100ms, std::bind(&PathPlanner::target_timer_callback, this));
    }

private:
    // ROS2 stuff...
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr target_pub_;
    rclcpp::TimerBase::SharedPtr target_pub_timer_;

    // ROS2 Callback functions
    void pcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg);
    void target_timer_callback();

    // Parameters
    float z_limit = 0.1;
    float leaf_size = 0.1;
    float yaw = 3.14;
    std::array<float, 3> position = {0.0, 0.0, 40.0};
};

void PathPlanner::pcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg) {
    // Convert point cloud message to pcl object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pointcloud_msg, *cloud);
    
    // Slice and remove points below - Crude ground extraction
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sliced(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> slicer;
    slicer.setInputCloud(cloud);
    slicer.setFilterFieldName("z");
    slicer.setFilterLimits(this->z_limit, std::numeric_limits<float>::max());
    slicer.filter(*cloud_sliced);
    
    // Downsample cloud using voxelfilter 
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxelGrid.setInputCloud(cloud_sliced);
    voxelGrid.setLeafSize(this->leaf_size, this->leaf_size, this->leaf_size);
    voxelGrid.filter(*filteredCloud);
    
    // Convert back to ROS2 msg for republishing (Visualization)
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*filteredCloud, output);
    output.header.frame_id = "lidar_frame";
    output.header.stamp = this->get_clock()->now();
    this->pcd_pub_->publish(output);
}

void PathPlanner::target_timer_callback() {
    /* Callback for target publishing */
    TrajectorySetpoint msg{};
    msg.position = this->position;
    msg.yaw = this->yaw;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000.0;
    this->target_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing setpoint...");
}







/* Main Function */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::Logger logger = rclcpp::get_logger("path_planner");
    RCLCPP_INFO_ONCE(logger, "Starting Path Planning Node...");
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}
