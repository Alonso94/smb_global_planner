cmake_minimum_required(VERSION 2.8.3)
project(smb_global_planner)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin_simple REQUIRED)
find_package(ompl REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)
include_directories(include)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/planner/voxblox_ompl_rrt.cpp
        src/planner/smb_global_planner.cpp
)
target_link_libraries(${PROJECT_NAME}
        ${OMPL_LIBRARIES}
        yaml-cpp
)

############
# BINARIES #
############
cs_add_executable(${PROJECT_NAME}_node
  src/smb_global_planner_node.cpp
)

cs_add_executable(test_node
  src/test.cpp
)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export(LIBRARIES ${OMPL_LIBRARIES})
