cmake_minimum_required(VERSION 3.5)
project(microstrain_inertial_driver)

# Locate the common code and messages
set(COMMON_NAME "microstrain_inertial_driver_common")
set(COMMON_DIR "${${PROJECT_NAME}_SOURCE_DIR}/${COMMON_NAME}")
set(COMMON_SRC_DIR "${COMMON_DIR}/src")
set(COMMON_INC_DIR "${COMMON_DIR}/include/${COMMON_NAME}")

# Make sure that the submodule has been properly initialized
if(NOT EXISTS "${COMMON_DIR}/src")
  message(STATUS "Initializing ${COMMON_DIR} submodule. This should only happen once.")

  # Make sure we can find the git executable
  find_package(Git)
  if(NOT Git_FOUND)
    message(FATAL_ERROR "Unable to initialize submodule because we could not find the git executable. Please clone this repo using 'git clone --recursive'")
  endif()

  # Initialize and update the submodule
  execute_process(
    WORKING_DIRECTORY "${${PROJECT_NAME}_SOURCE_DIR}"
    COMMAND ${CMAKE_COMMAND} -E env ${GIT_EXECUTABLE} submodule update --init --recursive
  )
endif()

# Download and install MSCL
include(${COMMON_DIR}/cmake/download_mscl.cmake)
download_mscl(VERSION "62.1.2")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-implicit-function-declaration -Wno-incompatible-pointer-types -Wno-format -fno-builtin-memcpy")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-format -fno-builtin-memcpy")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(microstrain_inertial_msgs REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
ament_export_include_directories()
ament_export_dependencies(
    roscpp
    rosidl_default_runtime
    cmake_modules
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    nav_msgs
    message_runtime
		microstrain_inertial_msgs
)

###########
## Build ##
###########

# Add the catkin includes
include_directories(${catkin_INCLUDE_DIRS} include ${COMMON_DIR}/include)

# Only lint the ROS2 code as the ROS1 code is linted through ROS1 and having two competing standards would make things messy quick
set(NODE_SRC_FILES
  src/microstrain_inertial_driver_node.cpp
)
if(BUILD_TESTING)
  # Include linting tests
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(${NODE_SRC_FILES} src/microstrain_inertial_driver.cpp include/${PROJECT_NAME}/microstrain_inertial_driver.h)
endif()

ament_package()

set(LIB_SRC_FILES
  ${COMMON_SRC_DIR}/microstrain_subscribers.cpp
  ${COMMON_SRC_DIR}/microstrain_publishers.cpp
  ${COMMON_SRC_DIR}/microstrain_node_base.cpp
  ${COMMON_SRC_DIR}/microstrain_services.cpp
  ${COMMON_SRC_DIR}/microstrain_parser.cpp
  ${COMMON_SRC_DIR}/microstrain_config.cpp
  src/microstrain_inertial_driver.cpp
)
set(LIB_INC_FILES
  ${COMMON_INC_DIR}/microstrain_subscribers.h
  ${COMMON_INC_DIR}/microstrain_publishers.h
  ${COMMON_INC_DIR}/microstrain_node_base.h
  ${COMMON_INC_DIR}/microstrain_services.h
  ${COMMON_INC_DIR}/microstrain_parser.h
  ${COMMON_INC_DIR}/microstrain_config.h
  ${COMMON_INC_DIR}/microstrain_defs.h
  ${COMMON_INC_DIR}/microstrain_ros_funcs.h
  include/${PROJECT_NAME}/microstrain_inertial_driver.h
)
add_library(${PROJECT_NAME} ${LIB_SRC_FILES} ${LIB_INC_FILES})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
  lifecycle_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  microstrain_inertial_msgs
)

# Executables
add_executable(${PROJECT_NAME}_node ${NODE_SRC_FILES} ${NODE_INC_FILES})

# Let the code know if it is being compiled with ROS1 or ROS2
if(DEFINED ENV{ROS_VERSION})
  add_definitions(-DMICROSTRAIN_ROS_VERSION=$ENV{ROS_VERSION})
else()
  message(FATAL_ERROR "ROS_VERSION environment variable is not set.")
endif()

# Allow the MSCL include directory to be accessed
include_directories(${MSCL_INC_PATH} ${BOOST_INC_PATH})

# Linking
target_link_libraries(${PROJECT_NAME}
  ${MSCL_LIB_PATH}
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  DESTINATION share/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# We also have to install MSCL and it's dependencies so it can be found at runtime
file(GLOB BOOST_LIBS "${MSCL_DIR}/Boost/lib/*")
install(FILES ${BOOST_LIBS} ${MSCL_DIR}/libmscl.so
  DESTINATION lib
)
