cmake_minimum_required(VERSION 3.0.2)
project(microstrain_inertial_examples)

# Install the launch and config files
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)