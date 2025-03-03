#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>

#include <skeleton_decomp.hpp>


class OcPlanner {

struct PathResult
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in; // Current lidar cloud 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered; // Current filtered cloud
};

public:

    /* Functions */
    void init();
    void OCPlan();
    void pcd_preprocess(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud);

    /* Data */
    PathResult PR;

private:
    /* Params */
    float ds_leaf_size = 0.5;

};