#ifndef EOD_ROBOT_EXCEPTION_H
#define EOD_ROBOT_EXCEPTION_H

#include <iostream>
#include <exception>

using namespace std;

/****************/
/*****故障码******/
/****************/
const string ERROR_DIVISION_ZERO = "Division by zero!";


/****************/
/****异常处理类****/
/****************/
class EodRobotException: public exception
{
public:
    EodRobotException(){};
    EodRobotException(string err);
    virtual ~EodRobotException() = default;
    virtual const char* what() const throw ();

private:
    string error;
};


#endif //EOD_ROBOT_EXCEPTION_H