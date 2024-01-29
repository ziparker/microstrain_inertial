/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Driver Definition File
// 
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2021, Parker Hannifin Corp
// 
// This code is licensed under MIT license (see LICENSE file for details)
// 
/////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef MICROSTRAIN_INERTIAL_DRIVER_MICROSTRAIN_INERTIAL_DRIVER_H_
#define MICROSTRAIN_INERTIAL_DRIVER_MICROSTRAIN_INERTIAL_DRIVER_H_

#include <cstdio>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <functional>

#include "microstrain_inertial_driver_common/node_common.h"

namespace microstrain 
{

///
/// \brief MicroStrainInertialDriverLifecycle class
///
class MicroStrainInertialDriverLifecycle : public rclcpp_lifecycle::LifecycleNode, public NodeCommon
{
 public:
  MicroStrainInertialDriverLifecycle();
  ~MicroStrainInertialDriverLifecycle() = default;

  bool configure_node();
  bool activate_node();
  bool deactivate_node();
  bool shutdown_or_cleanup_node();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &prev_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &prev_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &prev_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &prev_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &prev_state);

  void parse_and_publish_main_wrapper();
  void parse_and_publish_aux_wrapper();

 private:

  template<typename Object, void (Object::*Callback)()>
  RosTimerType create_timer_wrapper(double rate_hz);

  void handle_exception();
}; //MicroStrainInertialDriverLifecycle class

template<typename Object, void (Object::*Callback)()>
RosTimerType MicroStrainInertialDriverLifecycle::create_timer_wrapper(double rate_hz)
{
#ifdef MICROSTRAIN_ROLLING
  return createTimer(std::chrono::duration<double, std::milli>(1 / rate_hz), std::bind(Callback, this));
#else
  return createTimer<MicroStrainInertialDriverLifecycle>(node_, rate_hz, Callback, this);
#endif
}

} // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_MICROSTRAIN_INERTIAL_DRIVER_H_
