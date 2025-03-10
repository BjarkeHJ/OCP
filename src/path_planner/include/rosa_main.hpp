#include <iostream>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>

#include <Eigen/Dense>

class RosaPoints {

    struct rosa 
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pts_; 
        pcl::PointCloud<pcl::PointXYZ>::Ptr orig_pts_;
        pcl::PointCloud<pcl::Normal>::Ptr normals_;
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_w_normals;

        std::vector<std::vector<int>> neighs;
        std::vector<std::vector<int>> neighs_new;
        std::vector<std::vector<int>> surf_neighs;

        Eigen::MatrixXd vertices;
    };

public:
    /* Functions */
    void init();
    void rosa_main(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void set_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void adj_matrix(float &range_r);
    float pt_similarity_metric(pcl::PointXYZ &p1, pcl::Normal &v1, pcl::PointXYZ &p2, pcl::Normal &v2, float &range_r);
    void normalize();
    void normal_estimation();
    void rosa_calc();

    /* Data */
    rosa RC; // data structure for Rosa Cloud

private:
    /* Params */
    int ne_KNN = 10;
    int k_KNN = 10;
    int num_rosa_iter = 5;
    float th_dist = 0.01;
    float r_range = 0.1;

    /* Data */
    int pcd_size_;

};