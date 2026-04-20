#ifndef COBOT_HARDWARE__VISIBILITY_CONTROL_H_
#define COBOT_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then modified) from http://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COBOT_HARDWARE_EXPORT __attribute__ ((dllexport))
    #define COBOT_HARDWARE_IMPORT __attribute__ ((dllimport))
  #else
    #define COBOT_HARDWARE_EXPORT __declspec(dllexport)
    #define COBOT_HARDWARE_IMPORT __declspec(dllimport)
  #endif
  #ifdef COBOT_HARDWARE_BUILDING_DLL
    #define COBOT_HARDWARE_PUBLIC COBOT_HARDWARE_EXPORT
  #else
    #define COBOT_HARDWARE_PUBLIC COBOT_HARDWARE_IMPORT
  #endif
  #define COBOT_HARDWARE_PUBLIC_TYPE COBOT_HARDWARE_PUBLIC
  #define COBOT_HARDWARE_LOCAL
#else
  #define COBOT_HARDWARE_EXPORT __attribute__ ((visibility("default")))
  #define COBOT_HARDWARE_IMPORT
  #if __GNUC__ >= 4
    #define COBOT_HARDWARE_PUBLIC __attribute__ ((visibility("default")))
    #define COBOT_HARDWARE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define COBOT_HARDWARE_PUBLIC
    #define COBOT_HARDWARE_LOCAL
  #endif
  #define COBOT_HARDWARE_PUBLIC_TYPE
#endif

#endif  // COBOT_HARDWARE__VISIBILITY_CONTROL_H_
