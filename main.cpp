#include <iostream>

#include "meshslicer.h"

int main()
{
    std::cout << "Hello, World!" << std::endl;
    LoadModelAndMakeSlices("Data/15252_Key_Ring_Wall_Mount_Hand_v1.obj", glm::vec3(0,0,1), 0.1f);
    return 0;
}