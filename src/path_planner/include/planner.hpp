#include <chrono>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class PathPlanner : public rclcpp::Node {
    public:
        /* ROS2 Node */
        PathPlanner() : Node("path_planner") {
            /* QoS Profile */
            rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
            qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
            qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    
            /* ROS2 Subscribers */
            pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/isaac/point_cloud_0", 10,
                std::bind(&PathPlanner::pcdCallback,  this, std::placeholders::_1));
            odom_sub_ = this->create_subscription<VehicleOdometry>(
                "/fmu/out/vehicle_odometry", qos_profile, 
                std::bind(&PathPlanner::odomCallback, this, std::placeholders::_1));
            
            /* ROS2 Publishers */
            pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/RP_PointCloud", 10);
            target_pub_ = this->create_publisher<TrajectorySetpoint>("/target_setpoint", 10);
    
            /* ROS2 Timers */
            target_pub_timer_ = this->create_wall_timer(100ms, std::bind(&PathPlanner::target_timer_callback, this));
        }
    
    private:
        // ROS2 stuff...
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
        rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;
        rclcpp::Publisher<TrajectorySetpoint>::SharedPtr target_pub_;
        rclcpp::TimerBase::SharedPtr target_pub_timer_;
    
        // ROS2 Callback functions
        void pcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg);
        void odomCallback(const VehicleOdometry odom_msgs);
        void target_timer_callback();
    
        // Member functons
        void pcd_preprocess(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud);
        // void rosa();
    
        // Parameters
        float gnd_limit = 0.5;
        float ds_leaf_size = 0.5;
    
        float yaw = 3.14;
        std::array<float, 3> position = {5.0, 5.0, 40.0};
    
        //odometry
        std::array<float, 3> odom_pos;
        std::array<float, 4> odom_ori;
    
        // Point Cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud;
    };