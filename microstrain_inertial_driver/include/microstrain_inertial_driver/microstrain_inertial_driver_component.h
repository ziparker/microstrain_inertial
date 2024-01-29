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


#ifndef MICROSTRAIN_INERTIAL_DRIVER_MICROSTRAIN_INERTIAL_DRIVER_COMPONENT_H_
#define MICROSTRAIN_INERTIAL_DRIVER_MICROSTRAIN_INERTIAL_DRIVER_COMPONENT_H_

#include <cstdio>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <functional>

#include "microstrain_inertial_driver/microstrain_visibility.h"
#include "microstrain_inertial_driver_common/node_common.h"

namespace microstrain 
{

// TODO: Write a base class that both the lifecycle and node can share and then fully implement the component

///
/// \brief Microstrain class
///
class MicroStrainInertialDriverComponent : public rclcpp::Node, public NodeCommon
{
 public:
  MICROSTRAIN_PUBLIC MicroStrainInertialDriverComponent(const rclcpp::NodeOptions& options);
  ~MicroStrainInertialDriverComponent() = default;

  void parse_and_publish_main_wrapper();
  void parse_and_publish_aux_wrapper();

 private:

  template<typename Object, void (Object::*Callback)()>
  RosTimerType create_timer_wrapper(double rate_hz);
}; //Microstrain class

template<typename Object, void (Object::*Callback)()>
RosTimerType MicroStrainInertialDriverComponent::create_timer_wrapper(double rate_hz)
{
#ifdef MICROSTRAIN_ROLLING
  return createTimer(std::chrono::duration<double, std::milli>(1 / rate_hz), std::bind(Callback, this));
#else
  return createTimer<Object>(node_, rate_hz, Callback, this);
#endif
}

} // namespace microstrain

#endif  // MICROSTRAIN_INERTIAL_DRIVER_MICROSTRAIN_INERTIAL_DRIVER_COMPONENT_H_
