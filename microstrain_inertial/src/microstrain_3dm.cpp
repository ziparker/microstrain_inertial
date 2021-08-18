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
      RCLCPP_FATAL(this->get_logger(), "Failed to configure node base");
      return false;
    }
  }
  catch(mscl::Error_Connection)
  {
    RCLCPP_ERROR(this->get_logger(), "Error: Device Disconnected");
    return false;
  }

  catch(mscl::Error &e)
  {
    RCLCPP_FATAL(this->get_logger(), "Error: %s", e.what());
    return false;
  }


  return true;
} 


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Activate Node Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::activate_node()
{
  //Start timer callbacks
  std::chrono::milliseconds timer_interval_ms(static_cast<int>(1.0 / timer_update_rate_hz_ * 1000.0));
  
  //Create and stop the wall timers for data parsing and device status
  main_loop_timer_     = this->create_wall_timer(timer_interval_ms, std::bind(&MicrostrainNodeBase::parse_and_publish, this));

  RCLCPP_INFO(this->get_logger(), "Data Parsing timer started at <%f> hz", timer_update_rate_hz_);
  RCLCPP_INFO(this->get_logger(), "Resuming the device data streams");

  //Resume the device
  config_.inertial_device_->resume();
  
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
  
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Deactivate Node Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::deactivate_node()
{
  //Stop timer callbacks
  main_loop_timer_->cancel(); 
  device_status_timer_->cancel();

  RCLCPP_INFO(this->get_logger(), "Device set to idle");

  //Set the device to idle
  config_.inertial_device_->setToIdle();

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

  return true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Shutdown/Cleanup Node Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::shutdown_or_cleanup_node()
{

  //Release the inertial node, if necessary
  if(config_.inertial_device_)
  {
    config_.inertial_device_->setToIdle();
    config_.inertial_device_->connection().disconnect();
  }

  //Close raw data file if enabled
  if(config_.raw_file_enable_)
  {
    config_.raw_file_.close();
  }

  //Release timers
  main_loop_timer_.reset();
  device_status_timer_.reset();

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

  //Release services


  return true;
}

} // namespace Microstrain
