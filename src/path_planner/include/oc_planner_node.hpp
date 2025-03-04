#include <chrono>
#include <iostream>

#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <oc_planner.hpp>

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
    void odomCallback(const VehicleOdometry odom_msgs);
    void offboardControlTrigger(const std_msgs::msg::Bool::SharedPtr trigger);
    void target_timer_callback();
    void pcd_timer_callback();
    
    /* ROS2 Utils */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr target_pub_;
    rclcpp::TimerBase::SharedPtr target_pub_timer_;
    rclcpp::TimerBase::SharedPtr pcd_rp_timer_;
    
    /* Utils */
    std::unique_ptr<OcPlanner> PathPlanner;

    /* Data */
    std::array<float, 3> odom_pos;
    std::array<float, 4> odom_ori;


private:
    /* Params */
    bool trigger_flag_ = false;
    bool processing_ = false;

    // ---- TEMPORARY ----
    float yaw = 3.14;
    std::array<float, 3> position = {5.0, 5.0, 40.0};

};