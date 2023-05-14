cmake_minimum_required(VERSION 3.0.2)

project(planner)





find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs
    nav_msgs
    sensor_msgs
    visualization_msgs 
)



find_package(Eigen3 REQUIRED)

catkin_package(
     INCLUDE_DIRS include
    # LIBRARIES TEST_LIB
    # CATKIN_DEPENDS nav_mags roscpp  std_msgs sensor_msgs  visualization_msgs 
    # DEPENDS Eigen3
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIR} 
 
)


add_library(TEST_LIB
src/astar_planner.cpp
src/init_pose_subscriber.cpp
src/goal_pose_subscriber.cpp)

target_link_libraries(TEST_LIB 
${catkin_LIBRARIES}
)

add_executable(run_node 
src/run.cpp)

target_link_libraries(run_node TEST_LIB
)