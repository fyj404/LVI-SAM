cmake_minimum_required(VERSION 2.8.3)
project(lvi_sam)

################## 编译开关 compile switch##############
# -DIF_OFFICIAL=1: use origin official LVI-SAM code
# -DIF_OFFICIAL=0: use modified code of this repo
add_definitions(-DIF_OFFICIAL=0)


######################
### Cmake flags
######################
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

######################
### Packages
######################
find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    rospy
    roslib
    # msg
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    # cv
    cv_bridge
    # pcl
    pcl_conversions
    # msg generation
    message_generation
    octomap_ros
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system timer)
find_package(libxml2 QUIET)
if (NOT LIBXML2_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(Y REQUIRED libxml-2.0)
endif ()
set(CMAKE_PREFIX_PATH /usr/local/lib/cmake/proj/ ${CMAKE_PREFIX_PATH})
find_package(PROJ REQUIRED)

######################
### Message generation
######################
add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)
add_subdirectory(./ThirdParty/GeographicLib/)
######################
### Catkin
######################
catkin_package(
    DEPENDS PCL GTSAM
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${OCTOMAP_INCLUDE_DIR}
    ${PROJ_INCLUDE_DIRS}
    ${Y_INCLUDE_DIRS}
    ${LIBXML2_INCLUDE_DIRS}
    ./ThirdParty/GeographicLib/include/
)

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${OpenCV_LIBRARY_DIRS}
    ${GTSAM_LIBRARY_DIRS}
    ${OCTOMAP_LIBRARY_DIR}
    ${LIBXML2_LIBRARY_DIR}
    ${Y_LIBRARY_DIRS}
)
message(STATUS "Proj Include Dirs: ${PROJ_INCLUDE_DIRS}")
message(STATUS "Proj Libraries: ${PROJ_LIBRARIES}")
######################
### visual odometry
######################
file(GLOB visual_feature_files
    "src/visual_odometry/visual_feature/*.cpp"
    "src/visual_odometry/visual_feature/camera_models/*.cc"
)
file(GLOB visual_odometry_files
    "src/visual_odometry/visual_estimator/*.cpp"
    "src/visual_odometry/visual_estimator/factor/*.cpp"
    "src/visual_odometry/visual_estimator/initial/*.cpp"
    "src/visual_odometry/visual_estimator/utility/*.cpp"
)
file(GLOB visual_loop_files
    "src/visual_odometry/visual_loop/*.cpp"
    "src/visual_odometry/visual_loop/utility/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/DBoW/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/DUtils/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/DVision/*.cpp"
    "src/visual_odometry/visual_feature/camera_models/*.cc"
)
# Visual Feature Tracker
add_executable(${PROJECT_NAME}_visual_feature ${visual_feature_files})
target_link_libraries(${PROJECT_NAME}_visual_feature ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
# Visual Odometry
add_executable(${PROJECT_NAME}_visual_odometry ${visual_odometry_files})
target_link_libraries(${PROJECT_NAME}_visual_odometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}) 
# Visual Lopp
add_executable(${PROJECT_NAME}_visual_loop ${visual_loop_files})
target_link_libraries(${PROJECT_NAME}_visual_loop ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}) 

######################
### lidar odometry
######################

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/lidar_odometry/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam Boost::timer)
# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/lidar_odometry/imageProjection.cpp)
add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
# Feature Association
add_executable(${PROJECT_NAME}_featureExtraction src/lidar_odometry/featureExtraction.cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization src/lidar_odometry/mapOptmization.cpp)
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_include_directories(${PROJECT_NAME}_mapOptmization PRIVATE ${PROJ_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization PRIVATE ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} 
${OpenMP_CXX_FLAGS} gtsam Boost::timer  ${PROJ_LIBRARIES} libGeographic)


# Feature Association
add_executable(${PROJECT_NAME}_mapRegister src/lidar_odometry/mapRegister.cpp)
add_dependencies(${PROJECT_NAME}_mapRegister ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_mapRegister ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(${PROJECT_NAME}_gpsOdometry src/lidar_odometry/gpsOdometry.cpp)
add_dependencies(${PROJECT_NAME}_gpsOdometry ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_gpsOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} libGeographic)