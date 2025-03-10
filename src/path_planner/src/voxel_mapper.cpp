#include <voxel_mapper.hpp>

void VoxelMapper::init() {
    OCM.global_map.reset(new pcl::PointCloud<pcl::PointXYZ>);
    OCM.local_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    octree.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(tolerance));
}

void VoxelMapper::update_map() {
    std::lock_guard<std::mutex> lock(map_mutex);
    if (OCM.local_cloud->empty()) return;
    
    // Update voxel map with new information
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    octree->setInputCloud(OCM.global_map); // Global map for comparison
    octree->addPointsFromInputCloud();
    for (const auto &point : OCM.local_cloud->points) {
        std::vector<int> point_idx_radius_search;
        std::vector<float> point_radius_squared_distance;
        
        // Check if point is already contained in map
        if (!octree->radiusSearch(point, tolerance, point_idx_radius_search, point_radius_squared_distance)) {
            temp_cloud->points.push_back(point);
            octree->addPointToCloud(point, OCM.global_map);
        }
    }
    
    *OCM.global_map += *temp_cloud;
    // std::cout << "Current VoxelMap size: " << OCM.global_map->size() << std::endl;
}
