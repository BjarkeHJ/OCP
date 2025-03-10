#include <iostream> //for debugging

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/Dense> 

#include <rosa_main.hpp>
#include <voxel_mapper.hpp>

class OcPlanner {
    
    struct Pcloud
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered; // Current filtered cloud
    };

public:
    /* Functions */
    void init();
    void OCPlan();
    void get_segment();
    
    /* Params */
    float tolerance; 
    
    /* Data */
    Pcloud P;

    /* Utils */
    std::shared_ptr<VoxelMapper> VoxMap;
    std::shared_ptr<RosaPoints> skel_op;

private:
    /* Params */
    
    /* Data */
};
