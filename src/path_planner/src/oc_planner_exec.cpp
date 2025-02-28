#include <oc_planner_exec.hpp>
#include <skeleton_decomp.hpp>

void OcPlanner::pcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg) {
    // Convert point cloud message to pcl object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pointcloud_msg, *cloud);
    
    // Point Cloud Preprocessing
    pcd_preprocess(cloud);

    // Convert back to ROS2 msg for republishing (Visualization)
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*current_cloud, output);
    output.header.frame_id = "lidar_frame";
    output.header.stamp = this->get_clock()->now();
    this->pcd_pub_->publish(output);
}

void OcPlanner::odomCallback(const VehicleOdometry odom_msg) {
    /* Reviece odometry data and convert from NED to ENU */
    std::array<float, 3> temp = odom_msg.position;
    odom_pos = {temp[1], temp[0], -temp[2]};
    odom_ori = odom_msg.q;
}

void OcPlanner::target_timer_callback() {
    /* Callback for target publishing */
    TrajectorySetpoint msg{};
    msg.position = this->position;
    msg.yaw = this->yaw;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000.0;
    this->target_pub_->publish(msg);
}

void OcPlanner::pcd_preprocess(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud) {
    current_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // Uniform downsampling using VoxelGrid filter 
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(pointcloud);
    vg.setLeafSize(ds_leaf_size, ds_leaf_size, ds_leaf_size);
    vg.filter(*cloud_ds);
    
    // Ground Points Removal
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefs(new pcl::ModelCoefficients);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_gnd(new pcl::PointCloud<pcl::PointXYZ>);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ds_leaf_size); // Larger tolerance to remove points close to segmented plane
    seg.setInputCloud(cloud_ds);
    seg.segment(*inliers, *coefs);
    extract.setInputCloud(cloud_ds);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_no_gnd);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_no_gnd);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.5);
    sor.filter(*cloud_filtered);
    
    current_cloud = cloud_filtered;
}

/* Main Function */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::Logger logger = rclcpp::get_logger("path_planner");
    RCLCPP_INFO_ONCE(logger, "Starting Path Planning Node...");
    rclcpp::spin(std::make_shared<OcPlanner>());
    rclcpp::shutdown();
    return 0;
}
