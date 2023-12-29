#include <iostream>
#include <string>
#include "ConsoleInterface.h"
#include "Graphic_Task.h"

int main()
{
    ConsoleInterface inter;
    //inter.Work();
    Graphic_Task gt("MST_1.txt");
    gt.Work();
}

