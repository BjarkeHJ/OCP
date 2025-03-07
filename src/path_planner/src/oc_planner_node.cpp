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
    trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/trigger_", rclcpp::QoS(1).reliable().keep_last(1),
        std::bind(&OcPlannerNode::offboardControlTrigger, this, std::placeholders::_1));

    /* ROS2 Publishers */
    pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/RP_PointCloud", 10);
    target_pub_ = this->create_publisher<TrajectorySetpoint>("/target_setpoint", 10);
    voxel_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/VoxelMap", 10);

    /* ROS2 Timers */
    target_pub_timer_ = this->create_wall_timer(100ms, std::bind(&OcPlannerNode::target_timer_callback, this));
    pcd_rp_timer_ = this->create_wall_timer(10ms, std::bind(&OcPlannerNode::pcd_timer_callback, this));
    voxel_timer_ =  this->create_wall_timer(100ms, std::bind(&OcPlannerNode::voxel_timer_callback, this));
    run_timer_ = this->create_wall_timer(10ms, std::bind(&OcPlannerNode::run_timer_callback, this));

    /* ROS2 Transform */
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    /* Module Initialization */
    PathPlanner.reset(new OcPlanner);
    PathPlanner->init();
    VoxMap.reset(new VoxelMapper);
    VoxMap->init(tf_buffer_);

    /* Data Storage Initialization */
    current_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void OcPlannerNode::offboardControlTrigger(const std_msgs::msg::Bool::SharedPtr trigger) {
    /* Trigger when drone is armed and ready to takeoff */
    if (trigger->data && !offboard_control_trigger_) {
        RCLCPP_INFO(this->get_logger(), "Trigger received! Starting OCPlanner... ");
        offboard_control_trigger_ = true;
        trigger_sub_.reset(); //unsubscribe after first trigger command...
    }
}

void OcPlannerNode::pcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg) {
    pcl::fromROSMsg(*pointcloud_msg, *current_cloud); // Store current cloud ... 
}

void OcPlannerNode::target_timer_callback() {
    /* Callback for target publishing */
    TrajectorySetpoint msg{};
    msg.position = position_temp;
    msg.yaw = yaw_temp;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000.0;
    this->target_pub_->publish(msg);
}

void OcPlannerNode::pcd_timer_callback() {
    /* Publish current filtered point cloud */
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*PathPlanner->P.cloud_filtered, output);
    output.header.frame_id = "lidar_frame";
    output.header.stamp = this->get_clock()->now();
    this->pcd_pub_->publish(output);
}

void OcPlannerNode::voxel_timer_callback() {
    /* Publish Current Voxelmap */
    sensor_msgs::msg::PointCloud2 vm;
    pcl::toROSMsg(*VoxMap->OCM.global_map, vm);
    vm.header.frame_id = "odom";
    vm.header.stamp = this->get_clock()->now();
    voxel_pub_->publish(vm);
}

void OcPlannerNode::run_timer_callback() {
    /* Periodically check if OcPlanner is readdy - Execute if it is */
    if (offboard_control_trigger_ && run_trigger_) {
        OcRunner();
    }
}

void OcPlannerNode::OcRunner() {
    // Preprocess the current point cloud ...
    cloud_preprocess(current_cloud);
    // Update the Voxel Map ...
    VoxMap->update_map();
    // Plan according to Voxel Map and point cloud ...
    PathPlanner->OCPlan();
}

void OcPlannerNode::cloud_preprocess(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    // Uniform downsampling using VoxelGrid filter 
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(ds_leaf_size, ds_leaf_size, ds_leaf_size);
    vg.filter(*cloud_ds);
    
    // Outlier Removal based on density 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filt(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(cloud_ds);
    ror.setRadiusSearch(1.0); // Radius for checking neighbors
    ror.setMinNeighborsInRadius(7); // Minimum number of neighbors in the radius
    ror.filter(*cloud_filt);

    // Store filtered cloud for PathPlanning and VoxelMapping respectively
    PathPlanner->P.cloud_filtered = cloud_filt;
    VoxMap->OCM.local_cloud = cloud_filt;
}