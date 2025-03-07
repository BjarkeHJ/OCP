#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/passthrough.h>

#include <pcl_ros/transforms.hpp>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class VoxelMapper {

    struct OCMap {
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_map;
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_seg;
    };


public:
    /* Functions */
    void init(std::shared_ptr<tf2_ros::Buffer> tf_buffer);
    void update_map();

    /* Params */
    
    /* Utils */
    
    /* Data */
    OCMap OCM;
    

private:
    /* Params */
    float tolerance = 0.5; // Twice the downsample leaf size from pcd_preprocessing...
    
    /* Data */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    
};

