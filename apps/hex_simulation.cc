// Sample placeholder for more code to come. 

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/common/find_resource.h"

#include <iostream>
#include <fstream>
#include <string>

int main() {

    // some dumb test to check we can import models here - 
    const char phantomx[] =
"apps/"
"models/urdf/phantomx.urdf";
    std::cout << drake::FindResourceOrThrow(phantomx) << std::endl;
    return 0;
}