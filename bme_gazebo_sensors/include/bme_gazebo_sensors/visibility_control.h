#ifndef BME_GAZEBO_SENSORS__VISIBILITY_CONTROL_H_
#define BME_GAZEBO_SENSORS__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BME_GAZEBO_SENSORS_EXPORT __attribute__ ((dllexport))
    #define BME_GAZEBO_SENSORS_IMPORT __attribute__ ((dllimport))
  #else
    #define BME_GAZEBO_SENSORS_EXPORT __declspec(dllexport)
    #define BME_GAZEBO_SENSORS_IMPORT __declspec(dllimport)
  #endif
  #ifdef BME_GAZEBO_SENSORS_BUILDING_LIBRARY
    #define BME_GAZEBO_SENSORS_PUBLIC BME_GAZEBO_SENSORS_EXPORT
  #else
    #define BME_GAZEBO_SENSORS_PUBLIC BME_GAZEBO_SENSORS_IMPORT
  #endif
  #define BME_GAZEBO_SENSORS_PUBLIC_TYPE BME_GAZEBO_SENSORS_PUBLIC
  #define BME_GAZEBO_SENSORS_LOCAL
#else
  #define BME_GAZEBO_SENSORS_EXPORT __attribute__ ((visibility("default")))
  #define BME_GAZEBO_SENSORS_IMPORT
  #if __GNUC__ >= 4
    #define BME_GAZEBO_SENSORS_PUBLIC __attribute__ ((visibility("default")))
    #define BME_GAZEBO_SENSORS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BME_GAZEBO_SENSORS_PUBLIC
    #define BME_GAZEBO_SENSORS_LOCAL
  #endif
  #define BME_GAZEBO_SENSORS_PUBLIC_TYPE
#endif

#endif  // BME_GAZEBO_SENSORS__VISIBILITY_CONTROL_H_