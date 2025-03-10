#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/passthrough.h>

class VoxelMapper {

struct OCMap 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map;
};
    
    
public:
    /* Functions */
    void init();
    void update_map();
    
    /* Params */
    
    /* Data */
    OCMap OCM;
    float tolerance;
    
    /* Utils */
    std::unique_ptr<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>> octree;
    
    private:
    /* Params */
    
    /* Data */
    
    /* Utils */
    std::mutex map_mutex; // For thread safety

};  

