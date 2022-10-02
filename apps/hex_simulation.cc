// Sample placeholder for more code to come. 

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include <iostream>
#include <fstream>
#include <string>

int main() {

    // some dumb test to check we can import models here - 
    std::ifstream myfile ("models/urdf/phantomx.urdf");
    std::string mystring;
    if ( myfile.is_open() ) { // always check whether the file is open
    myfile >> mystring; // pipe file's content into stream
    std::cout << mystring; // pipe stream's content to standard output
    }
    return 0;
}