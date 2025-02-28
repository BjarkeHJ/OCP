#include <planner.hpp>

class PathPlanner : public rclcpp::Node {
public:
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

void PathPlanner::pcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg) {
    // Convert point cloud message to pcl object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pointcloud_msg, *cloud);
    
    // Point Cloud Preprocessing
    pcd_preprocess(cloud);

    // Convert back to ROS2 msg for republishing (Visualization)
    sensor_msgs::msg::PointCloud2 output;
    // pcl::toROSMsg(*current_cloud, output);
    pcl::toROSMsg(*current_cloud, output);
    output.header.frame_id = "lidar_frame";
    output.header.stamp = this->get_clock()->now();
    this->pcd_pub_->publish(output);
}

void PathPlanner::odomCallback(const VehicleOdometry odom_msg) {
    /* Reviece odometry data and convert from NED to ENU */
    std::array<float, 3> temp = odom_msg.position;
    odom_pos = {temp[1], temp[0], -temp[2]};
    odom_ori = odom_msg.q;
}

void PathPlanner::target_timer_callback() {
    /* Callback for target publishing */
    TrajectorySetpoint msg{};
    msg.position = this->position;
    msg.yaw = this->yaw;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000.0;
    this->target_pub_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "Publishing setpoint...");
}

void PathPlanner::pcd_preprocess(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud) {
    current_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // Uniform downsampling using VoxelGrid filter 
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(pointcloud);
    vg.setLeafSize(ds_leaf_size, ds_leaf_size, ds_leaf_size);
    vg.filter(*cloud_ds);
    
    std::cout << "Downsampld Point Cloud size: " << cloud_ds->points.size() << std::endl;

    // Ground Points Removal
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefs(new pcl::ModelCoefficients);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_gnd(new pcl::PointCloud<pcl::PointXYZ>);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(2 * ds_leaf_size); // Larger tolerance to remove points close to segmented plane
    seg.setInputCloud(cloud_ds);
    seg.segment(*inliers, *coefs);
    extract.setInputCloud(cloud_ds);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_no_gnd);
    
    current_cloud = cloud_no_gnd;
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
