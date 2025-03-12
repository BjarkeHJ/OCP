#include <rosa_main.hpp>

void RosaPoints::init() {
    RC.pts_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    RC.orig_pts_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    RC.normals_.reset(new pcl::PointCloud<pcl::Normal>);
    RC.cloud_w_normals.reset(new pcl::PointCloud<pcl::PointNormal>);

    good_points.reset(new pcl::PointCloud<pcl::PointXYZ>);

    pcd_size_ = 0;
}

void RosaPoints::rosa_main(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    if (cloud->points.size() < 10) {
        std::cout << "Recieved Cloud is Empty..." << std::endl;
        return;
    }

    set_cloud(cloud);

    normalize();

    pset.resize(pcd_size_, 3);
    vset.resize(pcd_size_, 3);
    vvar.resize(pcd_size_, 1);

    RC.datas = new double[pcd_size_ * 3]();
    // Insert normalized points
    for (int idx=0; idx<pcd_size_; idx++){
        RC.datas[idx] = RC.pts_->points[idx].x; 
        RC.datas[idx+pcd_size_] = RC.pts_->points[idx].y;
        RC.datas[idx+2*pcd_size_] = RC.pts_->points[idx].z;
    }

    adj_matrix(r_range);
    rosa_drosa();
}
    
void RosaPoints::set_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    // RC.orig_pts_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    // RC.normals_.reset(new pcl::PointCloud<pcl::Normal>);
    RC.pts_->clear();
    RC.normals_->clear();
    RC.orig_pts_ = cloud;
    pcd_size_ = RC.orig_pts_->points.size();
    RC.pts_mat.resize(pcd_size_, 3);
    RC.nrs_mat.resize(pcd_size_, 3);
    for (int i=0; i<pcd_size_; i++) {
        RC.pts_mat(i,0) = RC.orig_pts_->points[i].x;
        RC.pts_mat(i,1) = RC.orig_pts_->points[i].y;
        RC.pts_mat(i,2) = RC.orig_pts_->points[i].z;
    }
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
    else std::cout << "ROSA Warning: Normal Estimation - Point Cloud empty..." << std::endl;
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
    norm_scale = max_scale;

    // Normalize point cloud
    pcl::compute3DCentroid(*RC.pts_, centroid);
    for (int i=0; i<pcd_size_; i++) {
        RC.pts_->points[i].x = (RC.pts_->points[i].x - centroid(0)) / norm_scale;
        RC.pts_->points[i].y = (RC.pts_->points[i].y - centroid(1)) / norm_scale;
        RC.pts_->points[i].z = (RC.pts_->points[i].z - centroid(2)) / norm_scale;
    }

    // Estimate surface normals
    normal_estimation();

    RC.cloud_w_normals.reset(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*RC.pts_, *RC.normals_, *RC.cloud_w_normals);

    RC.pts_->clear();
    RC.normals_->clear();

    pcl::Normal normal;
    pcl::PointXYZ pt;
    for (int i=0; i<pcd_size_; ++i) {
        pt.x = RC.cloud_w_normals->points[i].x;
        pt.y = RC.cloud_w_normals->points[i].y;
        pt.z = RC.cloud_w_normals->points[i].z;
        normal.normal_x = -RC.cloud_w_normals->points[i].normal_x; 
        normal.normal_y = -RC.cloud_w_normals->points[i].normal_y; 
        normal.normal_z = -RC.cloud_w_normals->points[i].normal_z;
        RC.pts_->push_back(pt);
        RC.normals_->points.push_back(normal);
        RC.pts_mat(i,0) = RC.pts_->points[i].x;
        RC.pts_mat(i,1) = RC.pts_->points[i].y;
        RC.pts_mat(i,2) = RC.pts_->points[i].z;
        RC.nrs_mat(i,0) = RC.normals_->points[i].normal_x;
        RC.nrs_mat(i,1) = RC.normals_->points[i].normal_y;
        RC.nrs_mat(i,2) = RC.normals_->points[i].normal_z;
    }

    // saving point cloud w normals for analysis
    // pcl::PointCloud<pcl::PointNormal>::Ptr temp_pcl(new pcl::PointCloud<pcl::PointNormal>);
    // pcl::concatenateFields(*RC.pts_, *RC.normals_, *temp_pcl);
    // if (save_flag && !temp_pcl->empty()) {
    //     if (pcl::io::savePCDFile("output.pcd", *temp_pcl) == -1) {
    //         PCL_ERROR("Could not write file ...");
    //         return;
    //     }
    //     std::cout << "Saved point cloud ..." << std::endl;
    //     // save_flag = false;
    // }

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
    if (RC.pts_->empty()) {
        return;
    }

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
        v1 = RC.normals_->points[i];
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

void RosaPoints::rosa_drosa() {

    Extra_Del ed_; // Matrix helper functions

    rosa_initialize(RC.pts_, RC.normals_); // Initialized with normalized data (sets pset and vset)

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
        RC.surf_neighs.push_back(temp_surf); // Store neighbours of each point... RC.surf_neighs[i] = k_KNN nearest points
    }

    Eigen::Vector3d var_p, var_v, new_v;
    Eigen::MatrixXd indxs, extract_normals;
    for (int n=0; n<num_rosa_iter; n++) {
        std::cout << "drosa iteration: " << n << std::endl;

        Eigen::MatrixXd vnew = Eigen::MatrixXd::Zero(pcd_size_, 3);
        for (int pidx=0; pidx<pcd_size_; pidx++) {
            var_p = pset.row(pidx);
            var_v = vset.row(pidx);
            indxs = compute_active_samples(pidx, var_p, var_v);
            extract_normals = ed_.rows_ext_M(indxs, RC.nrs_mat);
            vnew.row(pidx) = compute_symmetrynormal(extract_normals).transpose();
            new_v = vnew.row(pidx);

            if (extract_normals.rows() > 0) {
                vvar(pidx, 0) = symmnormal_variance(new_v, extract_normals);
            }
            else {
                vvar(pidx,0) = 0.0;
            }
        }

        Eigen::MatrixXd offset(vvar.rows(), vvar.cols());
        offset.setOnes();
        offset = 0.00001 * offset;
        vvar = (vvar.cwiseAbs2() + offset).cwiseInverse();
        vset = vnew;
        /* Smoothing */
        std::vector<int> surf_;
        Eigen::MatrixXi snidxs;
        Eigen::MatrixXd snidxs_d, vset_ex, vvar_ex;
        for (int i=0; i<1; i++) {
            for (int p=0; p<pcd_size_; p++) {
                std::vector<int>().swap(surf_);
                surf_ = RC.surf_neighs[p]; // The the neighbours to point p
                snidxs.resize(surf_.size(), 1);
                snidxs = Eigen::Map<Eigen::MatrixXi>(surf_.data(), surf_.size(), 1);
                snidxs_d = snidxs.cast<double>(); vset_ex = ed_.rows_ext_M(snidxs_d, vset); vvar_ex = ed_.rows_ext_M(snidxs_d, vvar);
                vset.row(p) = symmnormal_smooth(vset_ex, vvar_ex);
            }
            vnew = vset;
        }
    }

    /* --- compute positions of ROSA --- */
    std::vector<int> poorIdx;
    pcl::PointCloud<pcl::PointXYZ>::Ptr goodPts (new pcl::PointCloud<pcl::PointXYZ>);
    std::map<Eigen::Vector3d, Eigen::Vector3d, Vector3dCompare> goodPtsPset;
    Eigen::Vector3d var_p_p, var_v_p, centroid;
    Eigen::MatrixXd indxs_p, extract_pts, extract_nrs;

    for (int pIdx=0; pIdx<pcd_size_; pIdx++) {
        var_p_p = pset.row(pIdx); 
        var_v_p = vset.row(pIdx);
        indxs_p = compute_active_samples(pIdx, var_p_p, var_v_p);

        //* Update Neighbors
        std::vector<int> temp_neigh;
        for (int p=0; p<(int)indxs_p.rows(); p++) {
            temp_neigh.push_back(indxs_p(p,0));
        }

        RC.neighs_new.push_back(temp_neigh);

        extract_pts = ed_.rows_ext_M(indxs_p, RC.pts_mat);
        extract_nrs = ed_.rows_ext_M(indxs_p, RC.nrs_mat);
        centroid = closest_projection_point(extract_pts, extract_nrs);

        if (abs(centroid(0)) < 1 && abs(centroid(1)) < 1 && abs(centroid(2)) < 1) {
            pset.row(pIdx) = centroid;
            pcl::PointXYZ goodPoint;
            Eigen::Vector3d goodPointP;
            goodPoint = RC.pts_->points[pIdx];
            goodPointP(0) = RC.pts_->points[pIdx].x; 
            goodPointP(1) = RC.pts_->points[pIdx].y; 
            goodPointP(2) = RC.pts_->points[pIdx].z;
            goodPts->points.push_back(goodPoint);
            goodPtsPset[goodPointP] = centroid;
        }
        else {
            poorIdx.push_back(pIdx);
        }
    }

    std::cout << "goodPts size: " << goodPts->points.size() << std::endl;

    if (goodPts->empty()) {
        std::cout << "goodPts empty" << std::endl;
        return;
    }

    // Temp...
    good_points = goodPts;

    rosa_tree.setInputCloud(goodPts);

    for (int pp=0; pp<(int)poorIdx.size(); pp++) {
      int pair = 1;
      pcl::PointXYZ search_point;
      search_point.x = RC.pts_->points[poorIdx[pp]].x; 
      search_point.y = RC.pts_->points[poorIdx[pp]].y; 
      search_point.z = RC.pts_->points[poorIdx[pp]].z;

      std::vector<int> pair_id(pair);
      std::vector<float> nn_squared_distance(pair);
      rosa_tree.nearestKSearch(search_point, pair, pair_id, nn_squared_distance);

      Eigen::Vector3d pairpos;
      pairpos(0) = goodPts->points[pair_id[0]].x;
      pairpos(1) = goodPts->points[pair_id[0]].y;
      pairpos(2) = goodPts->points[pair_id[0]].z;
      Eigen::Vector3d goodrp = goodPtsPset.find(pairpos)->second;
      pset.row(poorIdx[pp]) = goodrp;
    }

    dpset = pset;
}

void RosaPoints::rosa_initialize(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals) {
    Eigen::Matrix3d M;
    Eigen::Vector3d normal_v;

    for (int i=0; i<pcd_size_; i++) {
        pset(i,0) = cloud->points[i].x;
        pset(i,1) = cloud->points[i].y;
        pset(i,2) = cloud->points[i].z;

        normal_v(0) = normals->points[i].normal_x;
        normal_v(1) = normals->points[i].normal_y;
        normal_v(2) = normals->points[i].normal_z;

        M = create_orthonormal_frame(normal_v);
        vset.row(i) = M.row(1); // Extracts a vector orthogonal to normal_v...
    }
}

Eigen::Matrix3d RosaPoints::create_orthonormal_frame(Eigen::Vector3d &v) {
    // ! /* random process for generating orthonormal basis */
    v = v/v.norm();
    double TH_ZERO = 1e-10;
    // srand((unsigned)time(NULL));
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    M(0,0) = v(0); 
    M(0,1) = v(1); 
    M(0,2) = v(2);
    Eigen::Vector3d new_vec, temp_vec;

    // Seems inefficient to just iterate until satisfaction? - Rewrite using deterministic linear algebra (cross product method)?
    // The for outer for loops finds an orthonormal basis
    for (int i=1; i<3; ++i) {
      new_vec.setRandom();
      new_vec = new_vec/new_vec.norm();

      while (abs(1.0 - v.dot(new_vec)) < TH_ZERO) {
        // Run until vector (not too parallel) is found... Avoid colinear vectors
        new_vec.setRandom();
        new_vec = new_vec / new_vec.norm();
      }

      // Gramm-Schmidt process to find orthogonal vectors
      for (int j=0; j<i; ++j) {
        temp_vec = (new_vec - new_vec.dot(M.row(j)) * (M.row(j).transpose()));
        new_vec = temp_vec/temp_vec.norm();
      }

      M(i,0) = new_vec(0);
      M(i,1) = new_vec(1);
      M(i,2) = new_vec(2);
    }

    return M;
}

Eigen::MatrixXd RosaPoints::compute_active_samples(int &idx, Eigen::Vector3d &p_cut, Eigen::Vector3d &v_cut) {
    // Returns index 
    Eigen::MatrixXd out_indxs(pcd_size_, 1);
    int out_size = 0;
    std::vector<int> isoncut(pcd_size_, 0); // vector initialized with zeros

    pcloud_isoncut(p_cut, v_cut, isoncut, RC.datas, pcd_size_);
    
    std::vector<int> queue;
    queue.reserve(pcd_size_);
    queue.emplace_back(idx);

    int curr;
    while (!queue.empty()) {
        curr = queue.back();
        queue.pop_back();
        isoncut[curr] = 2;
        out_indxs(out_size++, 0) = curr;

        for (size_t i = 0; i < RC.neighs[curr].size(); ++i) {
            if (isoncut[RC.neighs[curr][i]] == 1) {
                isoncut[RC.neighs[curr][i]] = 3;
                queue.emplace_back(RC.neighs[curr][i]);
            }
        }
    }

    out_indxs.conservativeResize(out_size, 1); // Reduces the size down to an array of indices corresponding to the active samples
    return out_indxs;
}

void RosaPoints::pcloud_isoncut(Eigen::Vector3d& p_cut, Eigen::Vector3d& v_cut, std::vector<int>& isoncut, double*& datas, int& size) {
    DataWrapper data;
    data.factory(datas, size); // datas size is 3 x size

    std::vector<double> p(3); 
    p[0] = p_cut(0); 
    p[1] = p_cut(1); 
    p[2] = p_cut(2);
    std::vector<double> n(3); 
    n[0] = v_cut(0); 
    n[1] = v_cut(1); 
    n[2] = v_cut(2);
    distance_query(data, p, n, delta, isoncut);
}

void RosaPoints::distance_query(DataWrapper& data, const std::vector<double>& Pp, const std::vector<double>& Np, double delta, std::vector<int>& isoncut) {
    std::vector<double> P(3);
    
    // data.lenght() = pcd_size_
    for (int pIdx=0; pIdx < data.length(); pIdx++) {
        // retrieve current point
        data(pIdx, P);
        
        // check distance (fabs is floating point abs value...)
        // Np is normal vector to plane of point Pp. Distance is calculated as d = Np (dot) (Pp - P)
        if (fabs( Np[0]*(Pp[0]-P[0]) + Np[1]*(Pp[1]-P[1]) + Np[2]*(Pp[2]-P[2]) ) < delta) {
            isoncut[pIdx] = 1;
        }
    }
}

Eigen::Vector3d RosaPoints::compute_symmetrynormal(Eigen::MatrixXd& local_normals) {
    // This function determines the vector least variance amongst the local_normals. 
    // This can be interpreted as the "direction" of the skeleton inside the structure...

    Eigen::Matrix3d M; Eigen::Vector3d vec;
    double alpha = 0.0;
    int size = local_normals.rows();
    double Vxx, Vyy, Vzz, Vxy, Vyx, Vxz, Vzx, Vyz, Vzy;
    Vxx = (1.0+alpha)*local_normals.col(0).cwiseAbs2().sum()/size - pow(local_normals.col(0).sum(), 2)/pow(size, 2);
    Vyy = (1.0+alpha)*local_normals.col(1).cwiseAbs2().sum()/size - pow(local_normals.col(1).sum(), 2)/pow(size, 2);
    Vzz = (1.0+alpha)*local_normals.col(2).cwiseAbs2().sum()/size - pow(local_normals.col(2).sum(), 2)/pow(size, 2);
    Vxy = 2*(1.0+alpha)*(local_normals.col(0).cwiseProduct(local_normals.col(1))).sum()/size - 2*local_normals.col(0).sum()*local_normals.col(1).sum()/pow(size, 2);
    Vyx = Vxy;
    Vxz = 2*(1.0+alpha)*(local_normals.col(0).cwiseProduct(local_normals.col(2))).sum()/size - 2*local_normals.col(0).sum()*local_normals.col(2).sum()/pow(size, 2);
    Vzx = Vxz;
    Vyz = 2*(1.0+alpha)*(local_normals.col(1).cwiseProduct(local_normals.col(2))).sum()/size - 2*local_normals.col(1).sum()*local_normals.col(2).sum()/pow(size, 2);
    Vzy = Vyz;
    M << Vxx, Vxy, Vxz, Vyx, Vyy, Vyz, Vzx, Vzy, Vzz;

    Eigen::BDCSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d U = svd.matrixU();
    vec = U.col(M.cols()-1);

    return vec;
}

double RosaPoints::symmnormal_variance(Eigen::Vector3d& symm_nor, Eigen::MatrixXd& local_normals) {
    // This function computes the variance of the local normals
    Eigen::MatrixXd repmat; 
    Eigen::VectorXd alpha;
    int num = local_normals.rows();
    repmat.resize(num,3);

    for (int i=0; i<num; ++i) {
        repmat.row(i) = symm_nor;
    }

    alpha = local_normals.cwiseProduct(repmat).rowwise().sum();
    int n = alpha.size(); 
    double var;

    if (n>1) {
        var = (n+1)*(alpha.squaredNorm()/(n+1) - alpha.mean()*alpha.mean())/n;
    }
    else {
        var = alpha.squaredNorm()/(n+1) - alpha.mean()*alpha.mean();
    }

    return var;
}

Eigen::Vector3d RosaPoints::symmnormal_smooth(Eigen::MatrixXd& V, Eigen::MatrixXd& w) {
    Eigen::Matrix3d M; Eigen::Vector3d vec;
    double Vxx, Vyy, Vzz, Vxy, Vyx, Vxz, Vzx, Vyz, Vzy;
    Vxx = (w.cwiseProduct(V.col(0).cwiseAbs2())).sum();
    Vyy = (w.cwiseProduct(V.col(1).cwiseAbs2())).sum();
    Vzz = (w.cwiseProduct(V.col(2).cwiseAbs2())).sum();
    Vxy = (w.cwiseProduct(V.col(0)).cwiseProduct(V.col(1))).sum();
    Vyx = Vxy;
    Vxz = (w.cwiseProduct(V.col(0)).cwiseProduct(V.col(2))).sum();
    Vzx = Vxz;
    Vyz = (w.cwiseProduct(V.col(1)).cwiseProduct(V.col(2))).sum();
    Vzy = Vyz;
    M << Vxx, Vxy, Vxz, Vyx, Vyy, Vyz, Vzx, Vzy, Vzz;

    Eigen::BDCSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d U = svd.matrixU();
    vec = U.col(0);

    return vec;
}

Eigen::Vector3d RosaPoints::closest_projection_point(Eigen::MatrixXd& P, Eigen::MatrixXd& V) {
    Eigen::Vector3d vec;
    Eigen::VectorXd Lix2, Liy2, Liz2;

    Lix2 = V.col(0).cwiseAbs2();
    Liy2 = V.col(1).cwiseAbs2();
    Liz2 = V.col(2).cwiseAbs2();

    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    Eigen::Vector3d B = Eigen::Vector3d::Zero();

    M(0,0) = (Liy2+Liz2).sum();
    M(0,1) = -(V.col(0).cwiseProduct(V.col(1))).sum();
    M(0,2) = -(V.col(0).cwiseProduct(V.col(2))).sum();
    B(0) = (P.col(0).cwiseProduct(Liy2 + Liz2)).sum() - (V.col(0).cwiseProduct(V.col(1)).cwiseProduct(P.col(1))).sum() - (V.col(0).cwiseProduct(V.col(2)).cwiseProduct(P.col(2))).sum();
    M(1,0) = -(V.col(1).cwiseProduct(V.col(0))).sum();
    M(1,1) = (Lix2 + Liz2).sum();
    M(1,2) = -(V.col(1).cwiseProduct(V.col(2))).sum();
    B(1) = (P.col(1).cwiseProduct(Lix2 + Liz2)).sum() - (V.col(1).cwiseProduct(V.col(0)).cwiseProduct(P.col(0))).sum() - (V.col(1).cwiseProduct(V.col(2)).cwiseProduct(P.col(2))).sum();
    M(2,0) = -(V.col(2).cwiseProduct(V.col(0))).sum();
    M(2,1) = -(V.col(2).cwiseProduct(V.col(1))).sum();
    M(2,2) = (Lix2 + Liy2).sum();
    B(2) = (P.col(2).cwiseProduct(Lix2 + Liy2)).sum() - (V.col(2).cwiseProduct(V.col(0)).cwiseProduct(P.col(0))).sum() - (V.col(2).cwiseProduct(V.col(1)).cwiseProduct(P.col(1))).sum();

    if (std::abs(M.determinant()) < 1e-3) {
        vec << 1e8, 1e8, 1e8;
    }
    else {
        vec = M.inverse()*B;
    }

    return vec;
}