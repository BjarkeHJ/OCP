#include <oc_planner.hpp>

void OcPlanner::init() {
    std::cout << "OcPlanner Init Function..." << std::endl;
    
}

void OcPlanner::OCPlan() {
    
}

void OcPlanner::pcd_preprocess(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud) {
    // Uniform downsampling using VoxelGrid filter 
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(pointcloud);
    vg.setLeafSize(ds_leaf_size, ds_leaf_size, ds_leaf_size);
    vg.filter(*cloud_ds);
    
    // Ground Points Removal
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefs(new pcl::ModelCoefficients);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_gnd(new pcl::PointCloud<pcl::PointXYZ>);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ds_leaf_size); // Larger tolerance to remove points close to segmented plane
    seg.setInputCloud(cloud_ds);
    seg.segment(*inliers, *coefs);
    extract.setInputCloud(cloud_ds);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_no_gnd);

    PR.cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_no_gnd);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.5);
    sor.filter(*PR.cloud_filtered);
}

