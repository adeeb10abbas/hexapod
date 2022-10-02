// Sample placeholder for more code to come. 

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"

#include "drake/common/find_resource.h"
using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;

#include <iostream>
#include <fstream>
#include <string>

using namespace drake;
int main() {

  systems::DiagramBuilder<double> builder;

    MultibodyPlantConfig plant_config;
    plant_config.time_step = 0.01;
    plant_config.stiction_tolerance = 1.0E-3;
    plant_config.discrete_contact_solver = "sap";
      auto [plant, scene_graph] =
      multibody::AddMultibodyPlant(plant_config, &builder);
    // some dumb test to check we can import models here - 
    // const char phantomx[] =
    // "drake/hexapod/"
    // "models/urdf/phantomx.urdf";
    // std::cout << drake::FindResourceOrThrow(phantomx) << std::endl;
    const std::string full_name = "apps/models/urdf/phantomx.urdf";
    multibody::Parser(&plant).AddModelFromFile(full_name);
    return 0;
}
