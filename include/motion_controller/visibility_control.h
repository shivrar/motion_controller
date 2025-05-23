#ifndef MOTION_CONTROLLER__VISIBILITY_CONTROL_H_
#define MOTION_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MOTION_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define MOTION_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define MOTION_CONTROLLER_EXPORT __declspec(dllexport)
    #define MOTION_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef MOTION_CONTROLLER_BUILDING_LIBRARY
    #define MOTION_CONTROLLER_PUBLIC MOTION_CONTROLLER_EXPORT
  #else
    #define MOTION_CONTROLLER_PUBLIC MOTION_CONTROLLER_IMPORT
  #endif
  #define MOTION_CONTROLLER_PUBLIC_TYPE MOTION_CONTROLLER_PUBLIC
  #define MOTION_CONTROLLER_LOCAL
#else
  #define MOTION_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define MOTION_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define MOTION_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define MOTION_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MOTION_CONTROLLER_PUBLIC
    #define MOTION_CONTROLLER_LOCAL
  #endif
  #define MOTION_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // MOTION_CONTROLLER__VISIBILITY_CONTROL_H_
