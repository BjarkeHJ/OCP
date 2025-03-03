#include <oc_planner.hpp>

void OcPlanner::init() {
    P.cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>);
    P.pts_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    P.normals_.reset(new pcl::PointCloud<pcl::Normal>);
}

void OcPlanner::OCPlan() {
    pcd_preprocess();
    rosa();
}

void OcPlanner::pcd_preprocess() {
    // Uniform downsampling using VoxelGrid filter 
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(P.cloud_in);
    vg.setLeafSize(ds_leaf_size, ds_leaf_size, ds_leaf_size);
    vg.filter(*cloud_ds);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_gnd(new pcl::PointCloud<pcl::PointXYZ>);

    // Ground Points Removal
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefs(new pcl::ModelCoefficients);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(1.0); // Larger tolerance to remove points close to segmented plane
    seg.setInputCloud(cloud_ds);
    seg.segment(*inliers, *coefs);
    extract.setInputCloud(cloud_ds);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_no_gnd);

    // Statistical Outlier Removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_no_gnd);
    sor.setMeanK(100);
    sor.setStddevMulThresh(0.01);
    sor.filter(*P.cloud_filtered);
}

/* ROSA Computation */

void OcPlanner::rosa() {
    normalize();
}

void OcPlanner::normal_estimation() {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(P.cloud_filtered);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(ne_KNN);
    ne.compute(*cloud_normals);

    P.normals_.reset(new pcl::PointCloud<pcl::Normal>);
    P.normals_ = cloud_normals;
}

void OcPlanner::normalize() {
    // Store filtered point is P.pts_
    P.pts_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*P.cloud_filtered, *P.pts_);

    pcl::PointXYZ min, max;
    pcl::getMinMax3D(*P.pts_, min, max);
    
    float x_scale, y_scale, z_scale, max_scale;
    x_scale = max.x - min.x;
    y_scale = max.y - min.y;
    z_scale = max.z - min.z;

    if (x_scale >= y_scale) {
        max_scale = x_scale;
    }
    else max_scale = y_scale;
    if (max_scale < z_scale) {
        max_scale = z_scale;
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*P.pts_, centroid);

    for (int i=0; i<(int)P.pts_->points.size(); i++) {
        P.pts_->points[i].x = (P.pts_->points[i].x - centroid(0)) / max_scale;
        P.pts_->points[i].y = (P.pts_->points[i].y - centroid(1)) / max_scale;
        P.pts_->points[i].z = (P.pts_->points[i].z - centroid(2)) / max_scale;
    }

    normal_estimation();

    P.cloud_w_normals.reset(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*P.pts_, *P.normals_, *P.cloud_w_normals);

    pcd_size_ = P.cloud_w_normals->points.size();
    std::cout << "Point cloud size: " << pcd_size_ << "\n";
    P.pts_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    P.normals_.reset(new pcl::PointCloud<pcl::Normal>);
    pcl::PointXYZ pt;
    pcl::Normal normal;

    for (int i=0; i<pcd_size_; ++i) {
      pt.x = P.cloud_w_normals->points[i].x; pt.y = P.cloud_w_normals->points[i].y; pt.z = P.cloud_w_normals->points[i].z; 
      normal.normal_x = -P.cloud_w_normals->points[i].normal_x; 
      normal.normal_y = -P.cloud_w_normals->points[i].normal_y; 
      normal.normal_z = -P.cloud_w_normals->points[i].normal_z;
      P.pts_->points.push_back(pt);
      P.normals_->points.push_back(normal);
    }
}

void OcPlanner::rosa_comp() {
    std::vector<std::vector<int>>().swap(P.surf_neighs);
    pcl::KdTreeFLANN<pcl::PointXYZ> surf_kdtree;
    surf_kdtree.setInputCloud(P.pts_);

    std::vector<int> temp_surf(k_KNN);
    std::vector<float> nn_squared_distance(k_KNN);
    pcl::PointXYZ search_point_surf;

    for (int i=0; i<pcd_size_; i++) {
        std::vector<int>().swap(temp_surf);
        std::vector<float>().swap(nn_squared_distance);
        search_point_surf = P.pts_->points[i];
        surf_kdtree.nearestKSearch(search_point_surf, k_KNN, temp_surf, nn_squared_distance);
        P.surf_neighs.push_back(temp_surf);
    }

    for (int n=0; n<num_rosa_iter; n++) {
        Eigen::MatrixXd vnew = Eigen::MatrixXd::Zero(pcd_size_, 3);

    }

}



