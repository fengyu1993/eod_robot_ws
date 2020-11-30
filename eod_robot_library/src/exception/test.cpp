#include "eod_robot_exception.h"
#include <iostream>

int text(int a, int b)
{
    if (b == 0)
        throw EodRobotException(ERROR_DIVISION_ZERO);
    return a / b;
}

int main()
{
    try
    {
        text(1, 0);
    }
    catch(const exception& e)
    {
        cout << e.what() << endl;
    }
    
    return 0;
}