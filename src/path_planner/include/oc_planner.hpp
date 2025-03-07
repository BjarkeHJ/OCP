#include <iostream> //for debugging

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/Dense> 

class OcPlanner {
struct Pcloud
{
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
    void rosa();
    void normalize();
    void normal_estimation();
    void rosa_comp();

    /* Data */
    Pcloud P;
    
    /* Utils */

private:
    /* Params */
    float ds_leaf_size = 0.5;
    int ne_KNN = 10;
    int k_KNN = 10;
    int num_rosa_iter = 5;
    
    /* Data */
    int pcd_size_;
};
