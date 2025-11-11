#ifndef ACTION_CLEANING_ROBOT__VISIBILITY_CONTROL_H_
#define ACTION_CLEANING_ROBOT__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ACTION_CLEANING_ROBOT_EXPORT __attribute__ ((dllexport))
    #define ACTION_CLEANING_ROBOT_IMPORT __attribute__ ((dllimport))
  #else
    #define ACTION_CLEANING_ROBOT_EXPORT __declspec(dllexport)
    #define ACTION_CLEANING_ROBOT_IMPORT __declspec(dllimport)
  #endif
  #ifdef ACTION_CLEANING_ROBOT_BUILDING_DLL
    #define ACTION_CLEANING_ROBOT_PUBLIC ACTION_CLEANING_ROBOT_EXPORT
  #else
    #define ACTION_CLEANING_ROBOT_PUBLIC ACTION_CLEANING_ROBOT_IMPORT
  #endif
  #define ACTION_CLEANING_ROBOT_PUBLIC_TYPE ACTION_CLEANING_ROBOT_PUBLIC
  #define ACTION_CLEANING_ROBOT_LOCAL
#else
  #define ACTION_CLEANING_ROBOT_EXPORT __attribute__ ((visibility("default")))
  #define ACTION_CLEANING_ROBOT_IMPORT
  #if __GNUC__ >= 4
    #define ACTION_CLEANING_ROBOT_PUBLIC __attribute__ ((visibility("default")))
    #define ACTION_CLEANING_ROBOT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ACTION_CLEANING_ROBOT_PUBLIC
    #define ACTION_CLEANING_ROBOT_LOCAL
  #endif
  #define ACTION_CLEANING_ROBOT_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ACTION_CLEANING_ROBOT__VISIBILITY_CONTROL_H_
