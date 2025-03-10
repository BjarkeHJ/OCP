#include <oc_planner.hpp>

void OcPlanner::init() {
    /* Data structures initialization */
    P.cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>);

    /* Module initialization */
    VoxMap.reset(new VoxelMapper);
    VoxMap->tolerance = tolerance; // Set tolerance for voxelmapping (Before initialization!)
    VoxMap->init();

    skel_op.reset(new RosaPoints);
    skel_op->init();
}

void OcPlanner::OCPlan() {
    VoxMap->update_map();
    skel_op->rosa_main(P.cloud_filtered);
}

void OcPlanner::get_segment() {
    
}