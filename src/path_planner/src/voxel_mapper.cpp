#include <voxel_mapper.hpp>

void VoxelMapper::init(std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
    std::cout << "VoxelMapper Initialization" << std::endl;
    tf_buffer_ = tf_buffer;
    OCM.global_map.reset(new pcl::PointCloud<pcl::PointXYZ>);
    OCM.local_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    OCM.map_seg.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void VoxelMapper::update_map() {
    /* Updates Voxel Map */
    
    try {
        // Transform from lidar_frame to odom frame (sensor frame to world frame)
        if (OCM.local_cloud->empty()) {
            std::cout << "Warning: Point cloud empty - Skipping transformation..." << std::endl;
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        geometry_msgs::msg::TransformStamped transform_stamped;

        // Transform point cloud from lidar_frame to odom frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_ros::transformPointCloud("odom", *OCM.local_cloud, *temp_cloud, *tf_buffer_);

        // Ground points removal
        pcl::PointCloud<pcl::PointXYZ>::Ptr gnd_rm(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(temp_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.5, std::numeric_limits<float>::max());
        pass.filter(*gnd_rm);
      
        // transform_stamped = tf_buffer_->lookupTransform("lidar_frame", "odom", tf2::TimePointZero);
        // pcl_ros::transformPointCloud(*gnd_rm, *OCM.map_seg, transform_stamped);

        // Build octree 
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(tolerance);
        octree.setInputCloud(gnd_rm);
        octree.addPointsFromInputCloud();
        for (const auto &point : temp_cloud->points) {
            std::vector<int> point_idx_radius_search;
            std::vector<float> point_radius_squared_distance; 
            if (!octree.radiusSearch(point, tolerance, point_idx_radius_search, point_radius_squared_distance)) {
                transformed_cloud->points.push_back(point);
            }
        }

        // Add the processed point cloud back to the global map
        OCM.global_map->operator += (*transformed_cloud);
        std::cout << "Current VoxelMap size: " << OCM.global_map->size() << std::endl;
    }
    catch (tf2::TransformException &e) {
        std::cout << "Transform Failed: " << e.what() << std::endl;
        return;
    }
}

