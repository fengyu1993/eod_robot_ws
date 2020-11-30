#include "eod_robot_exception.h"

EodRobotException::EodRobotException(string err)
{
    error = err;
}

const char* EodRobotException::what() const throw ()
{
    return error.c_str();
}