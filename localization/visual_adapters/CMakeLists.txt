cmake_minimum_required(VERSION 3.16)
project(uav_visual_adapters)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
if(BACKEND STREQUAL "vins_fusion")
  find_package(Ceres REQUIRED)
  find_package(Boost REQUIRED COMPONENTS filesystem system)
  set(VINS "${UPSTREAM}/vins_estimator/src")
  file(GLOB CAMERA_SOURCES "${UPSTREAM}/camera_models/src/camera_models/*.cc")
  add_library(vins_core STATIC
    ${CAMERA_SOURCES}
    ${UPSTREAM}/camera_models/src/gpl/gpl.cc
    ${UPSTREAM}/camera_models/src/gpl/EigenQuaternionParameterization.cc
    ${UPSTREAM}/camera_models/src/sparse_graph/Transform.cc
    ${VINS}/estimator/parameters.cpp ${VINS}/estimator/estimator.cpp
    ${VINS}/estimator/feature_manager.cpp
    ${VINS}/factor/pose_local_parameterization.cpp
    ${VINS}/factor/projectionTwoFrameOneCamFactor.cpp
    ${VINS}/factor/projectionTwoFrameTwoCamFactor.cpp
    ${VINS}/factor/projectionOneFrameTwoCamFactor.cpp
    ${VINS}/factor/marginalization_factor.cpp
    ${VINS}/utility/utility.cpp
    ${VINS}/initial/solve_5pts.cpp ${VINS}/initial/initial_aligment.cpp
    ${VINS}/initial/initial_sfm.cpp ${VINS}/initial/initial_ex_rotation.cpp
    ${VINS}/featureTracker/feature_tracker.cpp
    vins_hooks.cpp)
  target_include_directories(vins_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} compat ${VINS} ${UPSTREAM}/camera_models/include
    ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
  target_compile_options(vins_core PRIVATE "SHELL:-include opencv2/imgproc/types_c.h"
    "SHELL:-include opencv2/calib3d/calib3d_c.h" "SHELL:-include opencv2/imgcodecs/legacy/constants_c.h")
  target_link_libraries(vins_core PUBLIC Ceres::ceres ${OpenCV_LIBS} ${Boost_LIBRARIES})
  add_executable(vins_fusion_node vins_node.cpp)
  ament_target_dependencies(vins_fusion_node rclcpp nav_msgs sensor_msgs std_msgs cv_bridge)
  target_link_libraries(vins_fusion_node vins_core)
  install(TARGETS vins_fusion_node DESTINATION lib/vins_fusion)
  add_executable(vins_hook_probe vins_hook_probe.cpp)
  target_link_libraries(vins_hook_probe vins_core)
  install(TARGETS vins_hook_probe DESTINATION lib/vins_fusion)
elseif(BACKEND STREQUAL "orb_slam3")
  find_package(Pangolin REQUIRED)
  add_executable(orb_slam3_node orb_node.cpp)
  # The upstream Release core uses -march=native. Eigen types cross this DSO
  # boundary, so the caller must use the same vectorization/alignment ABI.
  target_compile_options(orb_slam3_node PRIVATE -march=native)
  target_include_directories(orb_slam3_node PRIVATE ${UPSTREAM}/include
    ${UPSTREAM}/include/CameraModels ${UPSTREAM}/Thirdparty/Sophus ${UPSTREAM}
    ${EIGEN3_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})
  ament_target_dependencies(orb_slam3_node rclcpp nav_msgs sensor_msgs std_msgs cv_bridge)
  target_link_libraries(orb_slam3_node ${UPSTREAM}/lib/libORB_SLAM3.so
    ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
  install(TARGETS orb_slam3_node DESTINATION lib/orb_slam3)
  add_executable(orb_pose_probe orb_pose_probe.cpp)
  target_include_directories(orb_pose_probe PRIVATE ${EIGEN3_INCLUDE_DIRS})
  install(TARGETS orb_pose_probe DESTINATION lib/orb_slam3)
else()
  message(FATAL_ERROR "Select one real upstream core")
endif()
