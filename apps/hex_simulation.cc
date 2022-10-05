#include <iostream>
#include <fstream>
#include <string>

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"
#include "drake/common/find_resource.h"
#include "drake/systems/primitives/constant_vector_source.h"

using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using drake::math::RigidTransformd;
using drake::visualization::AddDefaultVisualization;
using Eigen::VectorXd;
using Eigen::Translation3d;


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

  // just trying to *see* something work now. 
    const math::RigidTransform<double> X_WF0 = math::RigidTransform<double>(
      math::RollPitchYaw(0.0, 0.0, 0.0), Eigen::Vector3d(0, 0, 0.2));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"), X_WF0);

  // Add model of the ground.
  const Vector4<double> green(0.5, 1.0, 0.5, 0.4);
  plant.RegisterVisualGeometry(plant.world_body(), RigidTransformd(),
                               geometry::HalfSpace(), "GroundVisualGeometry",
                               green);
  plant.Finalize();
  
  // Constant Source of zero actuation applied to make the system work for now. 
  auto constant_zero_source = builder.AddSystem<systems::ConstantVectorSource<double>>(VectorXd::Zero(plant.num_actuated_dofs()));
  constant_zero_source->set_name("Constant Zero Source");

  builder.Connect(constant_zero_source->get_output_port(), plant.get_actuation_input_port());

  // Add the visualization.
  AddDefaultVisualization(&builder);

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);
  
  //Simulate
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);
  simulator.get_mutable_context().SetAccuracy(1e-4);
  simulator.AdvanceTo(10);

  return 0;
}

int main() {
    doMain();
    return 0;
}