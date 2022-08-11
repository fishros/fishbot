
#pragma once

#ifdef WIN32
#ifdef ydlidar_IMPORTS
#define YDLIDAR_API __declspec(dllimport)
#else
#ifdef ydlidarStatic_IMPORTS
#define YDLIDAR_API
#else

#define YDLIDAR_API __declspec(dllexport)
#endif // YDLIDAR_STATIC_EXPORTS
#endif

#else
#define YDLIDAR_API
#endif // ifdef WIN32
