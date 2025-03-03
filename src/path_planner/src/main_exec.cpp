#include <oc_planner_node.hpp>

/* Main Executed Function */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::Logger logger = rclcpp::get_logger("path_planner");
    RCLCPP_INFO_ONCE(logger, "Starting Path Planning Node...");
    auto OCP_node = std::make_shared<OcPlannerNode>();
    rclcpp::spin(OCP_node);
    rclcpp::shutdown();
    return 0;
}