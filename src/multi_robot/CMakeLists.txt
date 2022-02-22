project(multi_robot)

cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_bt_actions REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(dependencies
rclcpp
rclcpp_action
plansys2_msgs
plansys2_domain_expert
plansys2_executor
plansys2_planner
plansys2_problem_expert
plansys2_pddl_parser
ament_index_cpp
plansys2_bt_actions
)

include_directories(include)

add_library(plansys2_close_gripper_bt_node SHARED src/behavior_tree_nodes/CloseGripper.cpp)
list(APPEND plugin_libs plansys2_close_gripper_bt_node)

add_library(plansys2_open_gripper_bt_node SHARED src/behavior_tree_nodes/OpenGripper.cpp)
list(APPEND plugin_libs plansys2_open_gripper_bt_node)

add_library(plansys2_lift_arm_bt_node SHARED src/behavior_tree_nodes/LiftArm.cpp)
list(APPEND plugin_libs plansys2_lift_arm_bt_node)

add_library(plansys2_lower_arm_bt_node SHARED src/behavior_tree_nodes/LowerArm.cpp)
list(APPEND plugin_libs plansys2_lower_arm_bt_node)

add_library(plansys2_extend_arm_bt_node SHARED src/behavior_tree_nodes/ExtendArm.cpp)
list(APPEND plugin_libs plansys2_extend_arm_bt_node)


foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(multi_robot_controller src/multi_robot_controller.cpp)
ament_target_dependencies(multi_robot_controller ${dependencies})

add_executable(approach_items_target_node src/approach_items_target_node.cpp)
ament_target_dependencies(approach_items_target_node ${dependencies})

#add_executable(drop_items_node src/drop_items_node.cpp)
#ament_target_dependencies(drop_items_node ${dependencies})

#add_executable(grab_items_node src/grab_items_node.cpp)
#ament_target_dependencies(grab_items_node ${dependencies})

add_executable(handle_items_node src/handle_items_node.cpp)
ament_target_dependencies(handle_items_node ${dependencies})

add_executable(approach_items_node src/approach_items_node.cpp)
ament_target_dependencies(approach_items_node ${dependencies})


install(
  DIRECTORY launch pddl config behavior_trees_xml params
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
 
  multi_robot_controller
  
  approach_items_target_node
  #drop_items_node

  #grab_items_node
  handle_items_node
  approach_items_node
 
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
