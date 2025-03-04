#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>

#include <Eigen/Dense> 


class OcPlanner {

struct Odometry
{
    std::array<float, 3> odom_pos;
    std::array<float, 4> odom_ori;
};


struct Pcloud
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in; // Current lidar cloud 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered; // Current filtered cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_w_normals; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr pts_; 
    pcl::PointCloud<pcl::Normal>::Ptr normals_;
    std::vector<std::vector<int>> neighs;
    std::vector<std::vector<int>> neighs_new;
    std::vector<std::vector<int>> surf_neighs;

    Eigen::MatrixXd vertices;
};

public:
    /* Functions */
    void init();
    void OCPlan();
    void pcd_preprocess();
    void rosa();
    void normalize();
    void normal_estimation();
    void rosa_comp();

    /* Data */
    Pcloud P;
    Odometry Odom;
    
    /* Utils */

private:
    /* Params */
    float ds_leaf_size = 0.2;
    int ne_KNN = 10;
    int k_KNN = 10;
    int num_rosa_iter = 5;
    
    /* Data */
    int pcd_size_;
};
