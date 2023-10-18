cmake_minimum_required(VERSION 3.8)
project(online_fgo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options( -Wall -Wextra -Wpedantic -std=c++17)
endif()

#set(CMAKE_CXX_FLAGS_RELEASE "-g -pthread")

# path for custom find scripts, use same of libRSF
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


# find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(irt_nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(ublox_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(message_filters REQUIRED)
find_package(TBB REQUIRED COMPONENTS tbb tbbmalloc)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)

# find other dependencies
find_package(Boost REQUIRED COMPONENTS system)
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
find_package(GTSAMCMakeTools)
# Ceres to solve the NLS problem
#find_package(Ceres 2.0 REQUIRED) # ToDo: actually we don't need ceres, but i just put it here
# Eigen for vectorized calculations
find_package(Eigen3 3.3.5 REQUIRED NO_MODULE)
# Simple multi-threading, ToDo: actually we also don't need this, cause roscpp should already have it
set(CMAKE_THREAD_PREFER_PTHREAD ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
# geographic lib for coordinate frame conversions

find_package(GeographicLib REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_storage_default_plugins REQUIRED)


set(AUTO_DIFF ON)

if (${AUTO_DIFF})
add_definitions(-DAUTO_DIFF)
endif()

# include
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

# include directories
include_directories(BEFORE "${PROJECT_SOURCE_DIR}")
include_directories(${GTSAM_INCLUDE_DIR}) ##

# for unittest scripts, only if we need
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")


# add all subdirectory
add_subdirectory(third_party)

set(USE_LIOSAM true)

include_directories(
    ${Boost_INCLUDE_DIR}
    third_party
    include
    include/${PROJECT_NAME}
)

add_library(${PROJECT_NAME} SHARED
        src/graph/GraphBase.cpp
        src/graph/GraphTimeCentric.cpp
        src/graph/GraphSensorCentric.cpp
        src/integrator/GNSSLCIntegrator.cpp
        src/integrator/GNSSTCIntegrator.cpp
        src/integrator/LIOIntegrator.cpp
        src/integrator/CorrevitIntegrator.cpp
        src/solver/FixedLagSmoother.cpp
        src/solver/BatchFixedLagSmoother.cpp
        src/solver/IncrementalFixedLagSmoother.cpp
        src/utils/GPutils.cpp
        src/utils/Pose3utils.cpp
        src/gnss_fgo/GNSSFGOTimeCentric.cpp
        src/gnss_fgo/GNSSFGOUrbanLoco.cpp
        src/gnss_fgo/GNSSFGOLooselyCoupled.cpp
        src/gnss_fgo/GNSSFGOBoreas.cpp
        src/gnss_fgo/GNSSFGOLocalizationBase.cpp)

# set target (libRSF) dependencies
target_link_libraries(${PROJECT_NAME}
    ${GeographicLib_LIBRARIES}
        gtsam
        gtsam_unstable
        Threads::Threads
        Eigen3::Eigen
        ${tools_lib_list}
        pthread
        tbb
        tbbmalloc
        ${PCL_LIBRARIES}
        ${OpenMP_CXX_FLAGS}
        #Ceres
        #gpslam
)

ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "std_msgs"
  "sensor_msgs"
  "irt_nav_msgs"
  "message_filters"
  "pluginlib"
  "nav_msgs"
  "PCL"
  "novatel_oem7_msgs"
  "ublox_msgs"
        "rosbag2_cpp"
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


if (${USE_LIOSAM})
  find_package(lio_sam REQUIRED)
  find_package(OpenCV REQUIRED)
  add_library(${PROJECT_NAME}_lidar SHARED
      src/sensor/lidar/LIOSAM.cpp)

  ament_target_dependencies(
          ${PROJECT_NAME}_lidar
          "rmw"
          "lio_sam"
          "rclcpp"
          "OpenCV"
          "irt_nav_msgs"
          "PCL"
          "visualization_msgs"
          "nav_msgs"
          "sensor_msgs"
          "std_msgs"
          "pluginlib"
          "tf2"
          "tf2_ros"
          "tf2_eigen"
          "tf2_sensor_msgs"
          "tf2_geometry_msgs"
  )
  target_link_libraries(${PROJECT_NAME}
        ${PROJECT_NAME}_lidar
  )
  target_compile_options(${PROJECT_NAME}_lidar PRIVATE ${OpenMP_CXX_FLAGS})
  target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_CXX_FLAGS})
  target_link_libraries(${PROJECT_NAME}_lidar  ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS})
endif ()

rclcpp_components_register_nodes(online_fgo "online_fgo::OnlineFGOGNSSINSIntegrationNode")
add_executable(online_fgo_node src/node/nodeOnlineFGOTimeCentric.cpp)
add_executable(urbanloco_node src/node/nodeOnlineFGOTimeCentricUrbanLoco.cpp)
add_executable(gt_node src/node/nodeOnlineFGOGT.cpp)
add_executable(boreas_node src/node/nodeOnlineFGOBoreas.cpp)

set(node_deps
    ${PROJECT_NAME}
    pthread
    tbb
    tbbmalloc
    ${GeographicLib_LIBRARIES}
    gtsam
    gtsam_unstable
    Threads::Threads
    Eigen3::Eigen
    #Ceres
)

set(node_dep_ros
    "rclcpp"
    "rclcpp_components"
    "rclcpp_lifecycle"
    "cv_bridge"
    "yaml_cpp_vendor"
    "rosidl_default_generators"
    "std_msgs"
    "sensor_msgs"
    "irt_nav_msgs"
    "pluginlib"
    "rmw"
    "rmw_implementation_cmake"
    "rmw_implementation"
    "rosbag2_cpp"
)

set(node_include_dir
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> ${GeographicLib_INCLUDE_DIRS}
)

target_link_libraries(online_fgo_node
        ${node_deps}
        ${OpenMP_CXX_FLAGS}
)
target_compile_options(online_fgo_node PRIVATE ${OpenMP_CXX_FLAGS})

target_include_directories(online_fgo_node PUBLIC
        ${node_include_dir}
        )
target_compile_features(online_fgo_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(online_fgo_node
  ${node_dep_ros}
)


target_link_libraries(urbanloco_node
        ${node_deps}
        ${OpenMP_CXX_FLAGS}
        )
target_compile_options(urbanloco_node PRIVATE ${OpenMP_CXX_FLAGS})

target_include_directories(urbanloco_node PUBLIC
        ${node_include_dir}
        )
target_compile_features(urbanloco_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(urbanloco_node
        ${node_dep_ros}
        )

target_link_libraries(gt_node
        ${node_deps}
        ${OpenMP_CXX_FLAGS}
        )
target_compile_options(gt_node PRIVATE ${OpenMP_CXX_FLAGS})

target_include_directories(gt_node PUBLIC
        ${node_include_dir}
        )
target_compile_features(gt_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(gt_node
        ${node_dep_ros}
        )

target_link_libraries(boreas_node
        ${node_deps}
        ${OpenMP_CXX_FLAGS}
        )
target_compile_options(boreas_node PRIVATE ${OpenMP_CXX_FLAGS})

target_include_directories(boreas_node PUBLIC
        ${node_include_dir}
        )
target_compile_features(boreas_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(boreas_node
        ${node_dep_ros}
        )


#############
## Install ##
#############
#idk if this helps us

pluginlib_export_plugin_description_file(${PROJECT_NAME} integrator_ros_plugins.xml)
ament_export_dependencies(${node_dep_ros})
ament_export_include_directories(include/${PROJECT_NAME} include)
ament_export_libraries(${PROJECT_NAME})


foreach(HDR ${fgo_hdrs})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
          FILES ${HDR}
          DESTINATION include/${DIR}
  )
endforeach()

install(TARGETS online_fgo_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS urbanloco_node
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS gt_node
        DESTINATION lib/${PROJECT_NAME})
install(TARGETS boreas_node
        DESTINATION lib/${PROJECT_NAME})
#install(TARGETS online_fgo_artus_node
#        DESTINATION lib/${PROJECT_NAME})

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin}
        )

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.py")
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.launch")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
