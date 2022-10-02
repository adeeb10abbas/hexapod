#include <iostream>
#include <fstream>
#include <string>

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"

#include "drake/common/find_resource.h"

using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;

using namespace drake; 

int doMain() {
  systems::DiagramBuilder<double> builder;
  MultibodyPlantConfig plant_config;
  plant_config.time_step = 0.01;
  plant_config.stiction_tolerance = 1.0E-3;
  plant_config.discrete_contact_solver = "sap";
    auto [plant, scene_graph] =
    multibody::AddMultibodyPlant(plant_config, &builder);

  const std::string full_name = "apps/models/urdf/phantomx.urdf";
  multibody::Parser(&plant).AddModelFromFile(full_name);
  return 0;
}

int main() {
    doMain();
    return 0;
}