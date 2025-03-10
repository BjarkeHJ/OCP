#include <rosa_main.hpp>

void RosaPoints::init() {
    RC.pts_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    RC.orig_pts_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcd_size_ = 0;
}

void RosaPoints::rosa_main(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    set_cloud(cloud);
    normalize();
    adj_matrix(r_range);
}
    
void RosaPoints::set_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    RC.orig_pts_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    RC.normals_.reset(new pcl::PointCloud<pcl::Normal>);
    RC.orig_pts_ = cloud;
    pcd_size_ = RC.orig_pts_->points.size();
}

void RosaPoints::normal_estimation() {
    if (!RC.pts_->empty()) {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(RC.pts_);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        ne.setKSearch(ne_KNN);
        ne.compute(*cloud_normals);

        RC.normals_ = cloud_normals;
    }
    else std::cout << "ROSA Normal Estimation: Point Cloud empty..." << std::endl;
}

void RosaPoints::normalize() {
    // Store original cloud ... 
    RC.pts_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*RC.orig_pts_, *RC.pts_);

    pcl::PointXYZ min, max;
    pcl::getMinMax3D(*RC.pts_, min, max);
    
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

    // Normalize point cloud
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*RC.pts_, centroid);
    for (int i=0; i<pcd_size_; i++) {
        RC.pts_->points[i].x = (RC.pts_->points[i].x - centroid(0)) / max_scale;
        RC.pts_->points[i].y = (RC.pts_->points[i].y - centroid(1)) / max_scale;
        RC.pts_->points[i].z = (RC.pts_->points[i].z - centroid(2)) / max_scale;
    }

    // Estimate surface normals
    normal_estimation();

    RC.cloud_w_normals.reset(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*RC.pts_, *RC.normals_, *RC.cloud_w_normals);

    // RC.normals_.reset(new pcl::PointCloud<pcl::Normal>);
    RC.normals_->clear();
    pcl::Normal normal;

    for (int i=0; i<pcd_size_; ++i) {
        normal.normal_x = -RC.cloud_w_normals->points[i].normal_x; 
        normal.normal_y = -RC.cloud_w_normals->points[i].normal_y; 
        normal.normal_z = -RC.cloud_w_normals->points[i].normal_z;
        RC.normals_->points.push_back(normal);
    }
}

float RosaPoints::pt_similarity_metric(pcl::PointXYZ &p1, pcl::Normal &v1, pcl::PointXYZ &p2, pcl::Normal &v2, float &range_r) {
    float Fs = 2.0;
    float k = 0.0;
    float dist, vec_dot, w;
    Eigen::Vector3d p1_, p2_, v1_, v2_;
    p1_ << p1.x, p1.y, p1.z;
    p2_ << p2.x, p2.y, p2.z;
    v1_ << v1.normal_x, v1.normal_y, v1.normal_z;
    v2_ << v2.normal_x, v2.normal_y, v2.normal_z;

    // distance similarity metric
    dist = (p1_ - p2_ + Fs*((p1_ - p2_).dot(v1_))*v1_).norm();
    dist = dist/range_r;
    if (dist <= 1) {
        k = 2*pow(dist,3) - 3*pow(dist,2) + 1;
    }
    vec_dot = v1_.dot(v2_);
    w = k*pow(std::max(0.0f, vec_dot), 2);
    return w;
}

void RosaPoints::adj_matrix(float &range_r) {
    RC.neighs.clear();
    RC.neighs.resize(pcd_size_);
    
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(RC.pts_); // Normalized points
    
    pcl::PointXYZ search_pt, p1, p2;
    pcl::Normal v1, v2;
    std::vector<int> indxs;
    std::vector<float> radius_squared_distance;
    float w1, w2, w;
    std::vector<std::vector<int>> pt_neighs_idx;

    for (int i=0; i<pcd_size_; i++) {
        // Do radius search for each point in current cloud ... 
        std::vector<int>().swap(indxs);
        std::vector<float>().swap(radius_squared_distance);
        p1 = RC.pts_->points[i];
        v2 = RC.normals_->points[i];
        search_pt = RC.pts_->points[i];
        tree.radiusSearch(search_pt, range_r, indxs, radius_squared_distance);
        std::vector<int> temp_neighs;

        for (int j=0; j<(int)indxs.size(); j++) {
            // For each neighbour compute the distance metric ...
            p2 = RC.pts_->points[indxs[j]];
            v2 = RC.normals_->points[indxs[j]];
            w1 = pt_similarity_metric(p1, v1, p2, v2, range_r);
            w2 = pt_similarity_metric(p2, v2, p1, v1, range_r);
            w = std::min(w1, w2);

            // Check if valid neighbour
            if (w > th_dist) {
                temp_neighs.push_back(indxs[j]);
            }
        }
        RC.neighs[i] = temp_neighs;
    }
}

void RosaPoints::rosa_calc() {
    std::vector<std::vector<int>>().swap(RC.surf_neighs);
    pcl::KdTreeFLANN<pcl::PointXYZ> surf_kdtree;
    surf_kdtree.setInputCloud(RC.pts_);

    std::vector<int> temp_surf(k_KNN);
    std::vector<float> nn_squared_distance(k_KNN);
    pcl::PointXYZ search_point_surf;

    for (int i=0; i<pcd_size_; i++) {
        std::vector<int>().swap(temp_surf);
        std::vector<float>().swap(nn_squared_distance);
        search_point_surf = RC.pts_->points[i];
        surf_kdtree.nearestKSearch(search_point_surf, k_KNN, temp_surf, nn_squared_distance);
        RC.surf_neighs.push_back(temp_surf);
    }

    for (int n=0; n<num_rosa_iter; n++) {
        Eigen::MatrixXd vnew = Eigen::MatrixXd::Zero(pcd_size_, 3);

    }
}