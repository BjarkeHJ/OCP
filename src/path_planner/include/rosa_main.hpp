#include <iostream>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>

#include <pcl/io/pcd_io.h> // for saving pcl tests

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <Extra_Del.hpp>

class DataWrapper {
private:
    double* data;
    int npoints;
    const static int ndim = 3; 
        
public: 
    void factory(double* data, int npoints ) {
        this->data = data;
        this->npoints = npoints;
    }

    /** 
     *  Data retrieval function
     *  @param a address over npoints
     *  @param b address over the dimensions
     */

    inline double operator()(int a, int b) {
        assert( a < npoints );
        assert( b < ndim );
        return data[ a + npoints*b ];
    }

    // retrieve a single point at offset a, in a vector (preallocated structure)
    inline void operator()(int a, std::vector<double>& p){
        assert( a < npoints );
        assert( (int)p.size() == ndim );
        p[0] = data[ a + 0*npoints ];
        p[1] = data[ a + 1*npoints ];
        p[2] = data[ a + 2*npoints ];
    }

    int length(){
        return this->npoints;
    }
};



class RosaPoints {
    struct Vector3dCompare 
    {
        bool operator()(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) const {
            if (v1(0) != v2(0)) return v1(0) < v2(0);
            if (v1(1) != v2(1)) return v1(1) < v2(1);
            return v1(2) < v2(2);
        }
    };
    
    struct rosa 
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pts_; 
        pcl::PointCloud<pcl::PointXYZ>::Ptr orig_pts_;
        pcl::PointCloud<pcl::Normal>::Ptr normals_;
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_w_normals;

        std::vector<std::vector<int>> neighs;
        std::vector<std::vector<int>> neighs_new;
        std::vector<std::vector<int>> surf_neighs;

        double *datas;
        Eigen::MatrixXd pts_mat; // cloud in matrix format
        Eigen::MatrixXd nrs_mat; // normals in matrix format

        Eigen::MatrixXd vertices;
    };

public:
    /* Functions */
    void init();
    void rosa_main(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    
    /* Data */
    rosa RC; // data structure for Rosa Cloud

    /* Temp... */
    pcl::PointCloud<pcl::PointXYZ>::Ptr good_points;
    
private:
    /* Params */
    int ne_KNN = 10;
    int k_KNN = 10;
    int num_rosa_iter = 5;
    float th_dist = 0.01;
    float r_range = 5.0;
    float delta = 0.01; // used for distance query in 

    /* Data */
    int pcd_size_;
    int seg_count;
    float norm_scale;
    Eigen::Vector4f centroid;
    Eigen::MatrixXd pset; // Stores normalized points for operations...
    Eigen::MatrixXd dpset;
    Eigen::MatrixXd vset; // Vectors orthogonal to the surface normals...  
    Eigen::MatrixXd vvar;
    pcl::PointCloud<pcl::PointXYZ>::Ptr skeleton_vertice_cloud;
    pcl::KdTreeFLANN<pcl::PointXYZ> rosa_tree;
    

    /* Functions */
    void set_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void adj_matrix(float &range_r);
    float pt_similarity_metric(pcl::PointXYZ &p1, pcl::Normal &v1, pcl::PointXYZ &p2, pcl::Normal &v2, float &range_r);
    void normalize();
    void normal_estimation();
    void rosa_drosa();
    void rosa_initialize(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals);
    Eigen::Matrix3d create_orthonormal_frame(Eigen::Vector3d &v);
    Eigen::MatrixXd compute_active_samples(int &idx, Eigen::Vector3d &p_cut, Eigen::Vector3d &v_cut);
    void pcloud_isoncut(Eigen::Vector3d& p_cut, Eigen::Vector3d& v_cut, std::vector<int>& isoncut, double*& datas, int& size);
    void distance_query(DataWrapper& data, const std::vector<double>& Pp, const std::vector<double>& Np, double delta, std::vector<int>& isoncut);
    Eigen::Vector3d compute_symmetrynormal(Eigen::MatrixXd& local_normals);
    double symmnormal_variance(Eigen::Vector3d& symm_nor, Eigen::MatrixXd& local_normals);
    Eigen::Vector3d symmnormal_smooth(Eigen::MatrixXd& V, Eigen::MatrixXd& w);
    Eigen::Vector3d closest_projection_point(Eigen::MatrixXd& P, Eigen::MatrixXd& V);


    /* Temporary */
    bool save_flag = true; // for saving instance of point cloud

};
