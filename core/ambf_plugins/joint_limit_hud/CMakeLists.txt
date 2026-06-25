# Reachability-verdict integration: the HUD subscribes to /cartesian_ik/reachability
# (published by the cartesian_ik reachability_probe_node) and renders a colour-coded
# "REACH:" line. AMBF already runs rclcpp in-process via ros_comm_plugin, so this only
# adds a small subscriber. find_package resolves because colcon builds AMBF with ROS
# sourced; rclcpp::rclcpp / ${diagnostic_msgs_TARGETS} carry their own include dirs.
#
# AMBF's old cmake_minimum_required (3.10) leaves these policies OLD, but ament's
# find_package machinery needs them NEW (CMP0057 -> if(IN_LIST); CMP0074 -> <pkg>_ROOT).
cmake_policy(SET CMP0057 NEW)
cmake_policy(SET CMP0074 NEW)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_library(joint_limit_hud SHARED joint_limit_hud.cpp)
target_link_libraries(joint_limit_hud ${AMBF_LIBRARIES} rclcpp::rclcpp ${diagnostic_msgs_TARGETS})
