#include <iostream>
#include <fstream>
#include <string>

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"

#include "drake/common/find_resource.h"

using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using drake::math::RigidTransformd;

using namespace drake; 

int doMain() {
  systems::DiagramBuilder<double> builder;
  MultibodyPlantConfig plant_config;
  plant_config.time_step = 0.01;
  plant_config.stiction_tolerance = 1.0E-3;
  plant_config.discrete_contact_solver = "sap";
    auto [plant, scene_graph] =
    multibody::AddMultibodyPlant(plant_config, &builder);

  const std::string full_name = "src/models/urdf/phantomx.urdf";
  multibody::Parser(&plant).AddModelFromFile(full_name);

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"));

    // Add model of the ground.
  // const double static_friction = 1.0;
  const Vector4<double> green(0.5, 1.0, 0.5, 1.0);

  plant.RegisterVisualGeometry(plant.world_body(), RigidTransformd(),
                               geometry::HalfSpace(), "GroundVisualGeometry",
                               green);
  
  const drake::multibody::Body<double>& base = plant.GetBodyByName("MP_BODY");
  DRAKE_DEMAND(base.is_floating());

  plant.Finalize();
  return 0;
}

int main() {
    doMain();
    return 0;
}