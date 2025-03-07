#include <chrono>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <oc_planner.hpp>
#include <voxel_mapper.hpp>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::unique_ptr;

class OcPlannerNode : public rclcpp::Node {
public:
    OcPlannerNode() : Node("oc_planner") {
        init();
    }
    
    /* Functions */
    void init();
    void pcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg);
    void offboardControlTrigger(const std_msgs::msg::Bool::SharedPtr trigger);
    void target_timer_callback();
    void pcd_timer_callback();
    void voxel_timer_callback();
    void run_timer_callback();
    void cloud_preprocess(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void OcRunner();
    
    /* ROS2 Utils */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr target_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_pub_;
    rclcpp::TimerBase::SharedPtr target_pub_timer_;
    rclcpp::TimerBase::SharedPtr pcd_rp_timer_;
    rclcpp::TimerBase::SharedPtr voxel_timer_;
    rclcpp::TimerBase::SharedPtr run_timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    /* Utils */
    std::unique_ptr<OcPlanner> PathPlanner;
    std::unique_ptr<VoxelMapper> VoxMap;

    /* Data */
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud;

private:
    /* Params */
    bool offboard_control_trigger_ = false;
    bool run_trigger_ = true;
    bool processing_ = false;
    float ds_leaf_size = 0.5;

    // ---- TEMPORARY ----
    float yaw_temp = 0.0;
    std::array<float, 3> position_temp = {3.0, 2.0, 45.0};

};