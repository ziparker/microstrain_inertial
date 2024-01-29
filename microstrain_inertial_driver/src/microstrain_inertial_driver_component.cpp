/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord ROS2 Inertial Driver Implementation File
// 
// Copyright (c) 2017, Brian Bingham
// Copyright (c)  2021, Parker Hannifin Corp
// 
// This code is licensed under MIT license (see LICENSE file for details)
// 
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <time.h>
#include <math.h>
#include <stdlib.h>

#include <ctime>
#include <string>
#include <vector>
#include <algorithm>

#include <tf2/LinearMath/Transform.h>

#include "microstrain_inertial_driver/microstrain_inertial_driver_component.h"

namespace microstrain
{

MicroStrainInertialDriverComponent::MicroStrainInertialDriverComponent(const rclcpp::NodeOptions& options) : rclcpp::Node("microstrain_inertial", options)
{
  // Configure the logger
#if MICROSTRAIN_ROLLING == 1 || MICROSTRAIN_HUMBLE == 1 || MICROSTRAIN_GALACTIC == 1
  auto debug_enable = std::getenv("MICROSTRAIN_INERTIAL_DEBUG");
  if (debug_enable != nullptr && std::string(debug_enable) == "true")
    get_logger().set_level(rclcpp::Logger::Level::Debug);
#else
  RCLCPP_INFO(this->get_logger(), "This version of ROS2 does not support changing the log level in C++");
#endif

  //Initialize the helper classes
  if (!NodeCommon::initialize(this))
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize base node");
  if (!NodeCommon::configure(this))
    RCLCPP_FATAL(this->get_logger(), "Failed to configure base node");
  if (!NodeCommon::activate())
    RCLCPP_FATAL(this->get_logger(), "Failed to activate base node");

  // Start a timer around a wrapper function to catch errors
  main_parsing_timer_ = createTimer<MicroStrainInertialDriverComponent>(node_, timer_update_rate_hz_, &MicroStrainInertialDriverComponent::parse_and_publish_main_wrapper, this);

  // Start the aux timer if we were requested to do so
  if (config_.aux_device_ != nullptr)
  {
    RCLCPP_INFO(this->get_logger(), "Starting aux port parsing");
    aux_parsing_timer_ = createTimer<MicroStrainInertialDriverComponent>(node_, 2.0, &MicroStrainInertialDriverComponent::parse_and_publish_aux_wrapper, this);
  }
}

void MicroStrainInertialDriverComponent::parse_and_publish_main_wrapper()
{
  // call the parsing function in a try catch block so we can transition the state instead of crashing when an error happens
  try
  {
    parseAndPublishMain();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error during main processing: %s", e.what());
    throw e;
  }
}

void MicroStrainInertialDriverComponent::parse_and_publish_aux_wrapper()
{
  // call the parsing function in a try catch block so we can transition the state instead of crashing when an error happens
  try
  {
    parseAndPublishAux();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error during aux processing: %s", e.what());
    throw e;
  }
}

} // namespace Microstrain

// Register our component to be able to be used as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(microstrain::MicroStrainInertialDriverComponent)