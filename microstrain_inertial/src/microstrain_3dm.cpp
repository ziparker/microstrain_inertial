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


/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <algorithm>
#include <time.h>
#include <math.h>
#include <vector>
#include <stdlib.h>
#include <ctime>

#include "microstrain_inertial/microstrain_3dm.h"
#include <tf2/LinearMath/Transform.h>
#include "lifecycle_msgs/msg/transition.hpp"

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization
/////////////////////////////////////////////////////////////////////////////////////////////////////

namespace microstrain
{

Microstrain::Microstrain() : rclcpp_lifecycle::LifecycleNode("ros2_mscl_node")
{
  //Initialize the helper classes
  if (!MicrostrainNodeBase::initialize(this))
    RCLCPP_FATAL(this->get_logger(), "Failed to initialize base node");

  //Declare/Initialize Parameters
  this->declare_parameter("port",                  std::string("/dev/ttyACM0"));
  this->declare_parameter("baudrate",              115200);
  this->declare_parameter("poll_port",             false);
  this->declare_parameter("poll_rate_hz",          1.0);
  this->declare_parameter("poll_max_tries",        60);
  this->declare_parameter("device_setup",          false);
  this->declare_parameter("save_settings",         false);
  this->declare_parameter("use_device_timestamp",  false);
  this->declare_parameter("use_enu_frame",         false);

  //IMU
  this->declare_parameter("publish_imu",           false);
  this->declare_parameter("publish_gps_corr",      false);
  this->declare_parameter("imu_data_rate",         10);
  this->declare_parameter("imu_orientation_cov",   DEFAULT_MATRIX);
  this->declare_parameter("imu_linear_cov",        DEFAULT_MATRIX);
  this->declare_parameter("imu_angular_cov",       DEFAULT_MATRIX);
  this->declare_parameter("imu_frame_id",          config_.imu_frame_id_);

  //GNSS 1/2
  this->declare_parameter("publish_gnss1",         false);
  this->declare_parameter("publish_gnss2",         false);
  this->declare_parameter("gnss1_data_rate",       1);
  this->declare_parameter("gnss2_data_rate",       1);
  this->declare_parameter("gnss1_antenna_offset",  DEFAULT_VECTOR);
  this->declare_parameter("gnss2_antenna_offset",  DEFAULT_VECTOR);
  this->declare_parameter("gnss1_frame_id",        config_.gnss_frame_id_[GNSS1_ID]);
  this->declare_parameter("gnss2_frame_id",        config_.gnss_frame_id_[GNSS2_ID]);

  //RTK Dongle configuration
  this->declare_parameter("rtk_dongle_enable", false);
  
  //Filter
  this->declare_parameter("publish_filter",             false);
  this->declare_parameter("filter_reset_after_config",  true);
  this->declare_parameter("filter_auto_init",           true);
  this->declare_parameter("filter_data_rate",           10);
  this->declare_parameter("filter_frame_id",            config_.filter_frame_id_);
  this->declare_parameter("publish_relative_position",  false);
  this->declare_parameter("filter_sensor2vehicle_frame_selector",                  0);
  this->declare_parameter("filter_sensor2vehicle_frame_transformation_euler",      DEFAULT_VECTOR);
  this->declare_parameter("filter_sensor2vehicle_frame_transformation_matrix",     DEFAULT_MATRIX);
  this->declare_parameter("filter_sensor2vehicle_frame_transformation_quaternion", DEFAULT_QUATERNION);

  this->declare_parameter("filter_initial_heading",          0.0);
  this->declare_parameter("filter_heading_source",           0x1);
  this->declare_parameter("filter_declination_source",       2);
  this->declare_parameter("filter_declination",              0.23);
  this->declare_parameter("filter_dynamics_mode",            1);
  this->declare_parameter("filter_pps_source",               1);
  this->declare_parameter("gps_leap_seconds",                18.0);
  this->declare_parameter("filter_angular_zupt",             false);
  this->declare_parameter("filter_velocity_zupt",            false);
  this->declare_parameter("filter_velocity_zupt_topic",      std::string("/moving_vel"));
  this->declare_parameter("filter_angular_zupt_topic",       std::string("/moving_ang"));
  this->declare_parameter("filter_external_gps_time_topic",  std::string("/external_gps_time"));
 
  //Additional GQ7 Filter
  this->declare_parameter("filter_adaptive_level" ,                   2);
  this->declare_parameter("filter_adaptive_time_limit_ms" ,           15000);
  this->declare_parameter("filter_enable_gnss_pos_vel_aiding",        true);
  this->declare_parameter("filter_enable_gnss_heading_aiding",        true);
  this->declare_parameter("filter_enable_altimeter_aiding",           false);
  this->declare_parameter("filter_enable_odometer_aiding",            false);
  this->declare_parameter("filter_enable_magnetometer_aiding",        false);
  this->declare_parameter("filter_enable_external_heading_aiding",    false);
  this->declare_parameter("filter_enable_external_gps_time_update",   false);
  this->declare_parameter("filter_enable_acceleration_constraint",    0);
  this->declare_parameter("filter_enable_velocity_constraint",        0);
  this->declare_parameter("filter_enable_angular_constraint",         0);
  this->declare_parameter("filter_init_condition_src",                0);
  this->declare_parameter("filter_auto_heading_alignment_selector",   0);
  this->declare_parameter("filter_init_reference_frame",              2);
  this->declare_parameter("filter_init_position",                     DEFAULT_VECTOR);   
  this->declare_parameter("filter_init_velocity",                     DEFAULT_VECTOR);
  this->declare_parameter("filter_init_attitude",                     DEFAULT_VECTOR);
  this->declare_parameter("filter_relative_position_frame",           2);
  this->declare_parameter("filter_relative_position_ref",             DEFAULT_VECTOR);   
  this->declare_parameter("filter_speed_lever_arm",                   DEFAULT_VECTOR);   
  this->declare_parameter("filter_enable_wheeled_vehicle_constraint", false);
  this->declare_parameter("filter_enable_vertical_gyro_constraint",   false);
  this->declare_parameter("filter_enable_gnss_antenna_cal",           false);
  this->declare_parameter("filter_gnss_antenna_cal_max_offset",       0.1);   

  //GPIO Configuration
  this->declare_parameter("gpio1_feature",   0);
  this->declare_parameter("gpio1_behavior",  0);
  this->declare_parameter("gpio1_pin_mode",  0);

  this->declare_parameter("gpio2_feature",   0);
  this->declare_parameter("gpio2_behavior",  0);
  this->declare_parameter("gpio2_pin_mode",  0);

  this->declare_parameter("gpio3_feature",   0);
  this->declare_parameter("gpio3_behavior",  0);
  this->declare_parameter("gpio3_pin_mode",  0);

  this->declare_parameter("gpio4_feature",   0);
  this->declare_parameter("gpio4_behavior",  0);
  this->declare_parameter("gpio4_pin_mode",  0);

  this->declare_parameter("gpio_config",     false);
  
  //Raw data file save
  this->declare_parameter("raw_file_enable",               false);
  this->declare_parameter("raw_file_include_support_data", false);
  this->declare_parameter("raw_file_directory",            std::string("."));
}



/////////////////////////////////////////////////////////////////////////////////////////////////////
// Configure State Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Microstrain::on_configure(const rclcpp_lifecycle::State &prev_state)
{
  //RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
 
  if(configure_node())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  else
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Activate State Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Microstrain::on_activate(const rclcpp_lifecycle::State &prev_state)
{
  //RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

  if(activate_node())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  else
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Deactivate State Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Microstrain::on_deactivate(const rclcpp_lifecycle::State &prev_state)
{
  //RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

  if(deactivate_node())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  else
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Cleanup State Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Microstrain::on_cleanup(const rclcpp_lifecycle::State &prev_state)
{
  //RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup() is called.");

  if(shutdown_or_cleanup_node())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  else
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
 // Shutdown State Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Microstrain::on_shutdown(const rclcpp_lifecycle::State &prev_state)
{
  //RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown() is called.");

  if(shutdown_or_cleanup_node())
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  else
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
// Configure Node Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::configure_node()
{
  ///////////////////////////////////////////////////////////////////////////
  //
  //Main loop setup
  ///
  ///////////////////////////////////////////////////////////////////////////
  try {
    RCLCPP_DEBUG(this->get_logger(), "Initializing base node");
    if (!MicrostrainNodeBase::configure(this))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to configure node base");
      return false;
    }
  }
  catch(const mscl::Error_Connection&)
  {
    RCLCPP_ERROR(this->get_logger(), "Error: Device Disconnected");
    return false;
  }

  catch(const mscl::Error &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
    return false;
  }


  return true;
} 


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Activate Node Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::activate_node()
{
  // Activate the base node to start the background tasks
  if (!MicrostrainNodeBase::activate())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to activate base node");
    return false;
  }

  // Start a timer around a wrapper function to catch errors
  parsing_timer_ = create_timer<Microstrain>(node_, timer_update_rate_hz_,
      &Microstrain::parse_and_publish_wrapper, this);
  device_status_timer_ = create_timer<Microstrain>(node_, 1.0,
      &Microstrain::device_status_wrapper, this);

  //Activate publishers
  if (publishers_.device_status_pub_)
    publishers_.device_status_pub_->on_activate();

  //IMU Publishers
  if(publishers_.imu_pub_)
    publishers_.imu_pub_->on_activate();
   
  if(publishers_.mag_pub_)
    publishers_.mag_pub_->on_activate();

  if(publishers_.gps_corr_pub_)
    publishers_.gps_corr_pub_->on_activate();
 
 
  //GNSS Publishers
  for(int i=0; i< NUM_GNSS; i++)
  {
    if(publishers_.gnss_pub_[i])
      publishers_.gnss_pub_[i]->on_activate();
  
    if(publishers_.gnss_odom_pub_[i])
      publishers_.gnss_odom_pub_[i]->on_activate();

    if(publishers_.gnss_time_pub_[i])
      publishers_.gnss_time_pub_[i]->on_activate();

    if(publishers_.gnss_aiding_status_pub_[i])
      publishers_.gnss_aiding_status_pub_[i]->on_activate();
  }

  //RTK Data publisher
  if(publishers_.rtk_pub_)
    publishers_.rtk_pub_->on_activate();
  
  //Filter Publishers
  if(publishers_.filter_status_pub_)
    publishers_.filter_status_pub_->on_activate();

  if(publishers_.filter_heading_pub_)
    publishers_.filter_heading_pub_->on_activate();

  if(publishers_.filter_heading_state_pub_)
    publishers_.filter_heading_state_pub_->on_activate();

  if(publishers_.filter_pub_)
    publishers_.filter_pub_->on_activate();

  if(publishers_.filtered_imu_pub_)
    publishers_.filtered_imu_pub_->on_activate();

  if(publishers_.filter_relative_pos_pub_)
    publishers_.filter_relative_pos_pub_->on_activate();
  
  if(publishers_.gnss_dual_antenna_status_pub_)
    publishers_.gnss_dual_antenna_status_pub_->on_activate();
  
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Deactivate Node Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::deactivate_node()
{
  //Deactivate the base node
  if (!MicrostrainNodeBase::deactivate())
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to deactivate node base");
  }

  //Deactivate publishers
  if (publishers_.device_status_pub_)
    publishers_.device_status_pub_->on_deactivate();
 
  //IMU Publishers
  if(publishers_.imu_pub_)
    publishers_.imu_pub_->on_deactivate();
   
  if(publishers_.mag_pub_)
    publishers_.mag_pub_->on_deactivate();

  if(publishers_.gps_corr_pub_)
    publishers_.gps_corr_pub_->on_deactivate();
 
 
  //GNSS Publishers
  for(int i=0; i< NUM_GNSS; i++)
  {
    if(publishers_.gnss_pub_[i])
      publishers_.gnss_pub_[i]->on_deactivate();
  
    if(publishers_.gnss_odom_pub_[i])
      publishers_.gnss_odom_pub_[i]->on_deactivate();

    if(publishers_.gnss_time_pub_[i])
      publishers_.gnss_time_pub_[i]->on_deactivate();

    if(publishers_.gnss_aiding_status_pub_[i])
      publishers_.gnss_aiding_status_pub_[i]->on_deactivate();
  }

  //RTK Data publisher
  if(publishers_.rtk_pub_)
    publishers_.rtk_pub_->on_deactivate();
  
  //Filter Publishers
  if(publishers_.filter_status_pub_)
    publishers_.filter_status_pub_->on_deactivate();

  if(publishers_.filter_heading_pub_)
    publishers_.filter_heading_pub_->on_deactivate();

  if(publishers_.filter_heading_state_pub_)
    publishers_.filter_heading_state_pub_->on_deactivate();

  if(publishers_.filter_pub_)
    publishers_.filter_pub_->on_deactivate();

  if(publishers_.filtered_imu_pub_)
    publishers_.filtered_imu_pub_->on_deactivate();

  if(publishers_.filter_relative_pos_pub_)
    publishers_.filter_relative_pos_pub_->on_deactivate();

  if(publishers_.gnss_dual_antenna_status_pub_)
    publishers_.gnss_dual_antenna_status_pub_->on_deactivate();

  return true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Shutdown/Cleanup Node Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::shutdown_or_cleanup_node()
{
  //Shutdown the base node
  if(!MicrostrainNodeBase::shutdown())
  {
    // Even though this is an error, don't return a failure on shutdown as it is not a fatal error
    RCLCPP_ERROR(this->get_logger(), "Failed to shutdown base node");
  }

  //Release publishers
  if (publishers_.device_status_pub_)
    publishers_.device_status_pub_.reset();

  //IMU Publishers
  if(publishers_.imu_pub_)
    publishers_.imu_pub_.reset();
   
  if(publishers_.mag_pub_)
    publishers_.mag_pub_.reset();

  if(publishers_.gps_corr_pub_)
    publishers_.gps_corr_pub_.reset();
 
 
  //GNSS Publishers
  for(int i=0; i< NUM_GNSS; i++)
  {
    if(publishers_.gnss_pub_[i])
      publishers_.gnss_pub_[i].reset();
  
    if(publishers_.gnss_odom_pub_[i])
      publishers_.gnss_odom_pub_[i].reset();

    if(publishers_.gnss_time_pub_[i])
      publishers_.gnss_time_pub_[i].reset();

    if(publishers_.gnss_aiding_status_pub_[i])
      publishers_.gnss_aiding_status_pub_[i].reset();
  }

  //RTK Data publisher
  if(publishers_.rtk_pub_)
    publishers_.rtk_pub_.reset();
  
  //Filter Publishers
  if(publishers_.filter_status_pub_)
    publishers_.filter_status_pub_.reset();

  if(publishers_.filter_heading_pub_)
    publishers_.filter_heading_pub_.reset();

  if(publishers_.filter_heading_state_pub_)
    publishers_.filter_heading_state_pub_.reset();

  if(publishers_.filter_pub_)
    publishers_.filter_pub_.reset();

  if(publishers_.filtered_imu_pub_)
    publishers_.filtered_imu_pub_.reset();

  if(publishers_.filter_relative_pos_pub_)
    publishers_.filter_relative_pos_pub_.reset();

  if(publishers_.gnss_dual_antenna_status_pub_)
    publishers_.gnss_dual_antenna_status_pub_.reset();

  return true;
}

void Microstrain::parse_and_publish_wrapper()
{
  // call the parsing function in a try catch block so we can transition the state instead of crashing when an error happens
  try
  {
    parseAndPublish();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error during processing: %s", e.what());
    handle_exception();
  }
}

void Microstrain::device_status_wrapper()
{
  // call the device status function in a try catch block so we can transition the state instead of crashing when an error happens
  try
  {
    publishers_.publishDeviceStatus();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error during processing: %s", e.what());
    handle_exception();
  }
}

void Microstrain::handle_exception()
{
  // Manuallly transition to deactivate state so that the node can be cleanly restarted
  RCLCPP_INFO(this->get_logger(), "Transitioning to deactivate state");
  const auto& inactive_state = LifecycleNode::deactivate();
  if (inactive_state.label() == "inactive")
  {
    RCLCPP_WARN(this->get_logger(), "Successfully transitioned to inactive, cleaning up node to fresh state");
    const auto& cleanup_state = LifecycleNode::cleanup();
    if (cleanup_state.label() == "unconfigured")
    {
      RCLCPP_WARN(this->get_logger(), "Node has been successfully cleaned up from error. transition to configure state to reconfigure");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Transition to cleanup resulted in transition to %s instead of inactive", cleanup_state.label().c_str());
      RCLCPP_ERROR(this->get_logger(), "Unable to recover, so transitioning to shutdown. This node is no longer usable");
      LifecycleNode::shutdown();
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Transition to deactivate resulted in transition to %s instead of inactive", inactive_state.label().c_str());
    RCLCPP_ERROR(this->get_logger(), "Unable to recover, so transitioning to shutdown. This node is no longer usable");
    LifecycleNode::shutdown();
  }
}

} // namespace Microstrain
