#include <oc_planner_node.hpp>

void OcPlannerNode::init() {
    
    /* QoS Profile */
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    
    /* ROS2 Subscribers */
    pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/isaac/point_cloud_0", 10,
        std::bind(&OcPlannerNode::pcdCallback,  this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos_profile, 
        std::bind(&OcPlannerNode::odomCallback, this, std::placeholders::_1));
    trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/trigger_", rclcpp::QoS(1).reliable().keep_last(1),
        std::bind(&OcPlannerNode::offboardControlTrigger, this, std::placeholders::_1));

    /* ROS2 Publishers */
    pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/RP_PointCloud", 10);
    target_pub_ = this->create_publisher<TrajectorySetpoint>("/target_setpoint", 10);

    /* ROS2 Timers */
    target_pub_timer_ = this->create_wall_timer(100ms, std::bind(&OcPlannerNode::target_timer_callback, this));
    pcd_rp_timer_ = this->create_wall_timer(100ms, std::bind(&OcPlannerNode::pcd_timer_callback, this));
    
    /* Module Initialization */
    PathPlanner.reset(new OcPlanner);
    PathPlanner->init();
}

void OcPlannerNode::offboardControlTrigger(const std_msgs::msg::Bool::SharedPtr trigger) {
    if (trigger->data && !trigger_flag_) {
        RCLCPP_INFO(this->get_logger(), "Trigger received! Starting OCPlanner... ");
        trigger_flag_ = true;
        PathPlanner->OCPlan();
        trigger_sub_.reset(); //unsubscribe after first trigger command...
    }
}

void OcPlannerNode::pcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg) {
    // Convert point cloud message to pcl object
    PathPlanner->PR.cloud_in.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pointcloud_msg, *PathPlanner->PR.cloud_in);

    PathPlanner->pcd_preprocess(PathPlanner->PR.cloud_in);
}

void OcPlannerNode::odomCallback(const VehicleOdometry odom_msg) {
    /* Reviece odometry data and convert from NED to ENU */
    std::array<float, 3> temp = odom_msg.position;
    odom_pos = {temp[1], temp[0], -temp[2]};
    odom_ori = odom_msg.q;
}

void OcPlannerNode::target_timer_callback() {
    /* Callback for target publishing */
    TrajectorySetpoint msg{};
    msg.position = this->position;
    msg.yaw = this->yaw;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000.0;
    this->target_pub_->publish(msg);
}

void OcPlannerNode::pcd_timer_callback() {
    // Convert back to ROS2 msg for republishing (Visualization)
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*PathPlanner->PR.cloud_filtered, output);
    output.header.frame_id = "lidar_frame";
    output.header.stamp = this->get_clock()->now();
    this->pcd_pub_->publish(output);
}
