//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#ifndef MAV_SIMULATOR_OPENCV_HELPERS_H
#define MAV_SIMULATOR_OPENCV_HELPERS_H
int ignoreError( int status, const char* func_name,
                 const char* err_msg, const char* file_name,
                 int line, void* userdata )
{
  return 0;   //Return non zero value to terminate program
}
#endif //MAV_SIMULATOR_OPENCV_HELPERS_H
