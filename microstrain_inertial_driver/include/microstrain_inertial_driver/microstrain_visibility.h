// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MICROSTRAIN_INERTIAL_DRIVER_VISIBILITY_H_
#define MICROSTRAIN_INERTIAL_DRIVER_VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define MICROSTRAIN_EXPORT __attribute__ ((dllexport))
    #define MICROSTRAIN_IMPORT __attribute__ ((dllimport))
  #else
    #define MICROSTRAIN_EXPORT __declspec(dllexport)
    #define MICROSTRAIN_IMPORT __declspec(dllimport)
  #endif

  #ifdef MICROSTRAIN_DLL
    #define MICROSTRAIN_PUBLIC MICROSTRAIN_EXPORT
  #else
    #define MICROSTRAIN_PUBLIC MICROSTRAIN_IMPORT
  #endif

  #define MICROSTRAIN_PUBLIC_TYPE MICROSTRAIN_PUBLIC

  #define MICROSTRAIN_LOCAL

#else

  #define MICROSTRAIN_EXPORT __attribute__ ((visibility("default")))
  #define MICROSTRAIN_IMPORT

  #if __GNUC__ >= 4
    #define MICROSTRAIN_PUBLIC __attribute__ ((visibility("default")))
    #define MICROSTRAIN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MICROSTRAIN_PUBLIC
    #define MICROSTRAIN_LOCAL
  #endif

  #define MICROSTRAIN_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // MICROSTRAIN_INERTIAL_DRIVER_VISIBILITY_H_