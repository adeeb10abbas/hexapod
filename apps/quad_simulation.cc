#include <iostream>
#include <fstream>
#include <string>

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config_functions.h"
#include "drake/common/find_resource.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/common/text_logging.h"
#include "drake/common/eigen_types.h"

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

  const std::string full_name = "apps/models/urdf/a1.urdf";
  multibody::Parser(&plant).AddModelFromFile(full_name);
  // Add model of the ground.
  const Vector4<double> green(0.0, 0.0, 0.0, 0.2);
  plant.RegisterVisualGeometry(plant.world_body(), RigidTransformd(),
                               geometry::HalfSpace(), "GroundVisualGeometry",
                               green);
  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  // World pose of the base frame of the model.
  const auto X_WorldBase = plant.GetFrameByName("WorldBody").CalcPoseInWorld(*plant_context);
  const math::RigidTransform<double> X_WF0 = math::RigidTransform<double>(
   math::RollPitchYaw(0.0, 0.0, 0.0), Eigen::Vector3d(0, 0, 0.65));

  auto X_WF = X_WorldBase * X_WF0;

  plant.SetDefaultFreeBodyPose(plant.GetBodyByName("base"), X_WF);


  
  // Constant Source of zero actuation applied to make the system work for now. 
  auto constant_zero_source = builder.AddSystem<systems::ConstantVectorSource<double>>(
    VectorXd::Zero(plant.num_actuated_dofs()));
  
  std::cout << "num_actuated_dofs: " << plant.num_actuated_dofs() << std::endl;
  std::cout << "num_positions: " << plant.num_positions() << std::endl;
  std::cout << "num_velocities: " << plant.num_velocities() << std::endl;
  std::cout << "num_multibody_states: " << plant.num_multibody_states() << std::endl;
  std::cout << "num_actuators: " << plant.num_actuators() << std::endl;
  


  int num_q = plant.num_positions();

  std::vector<std::string> joint_names;
  for (int i = 0; i < plant.num_actuators(); i++)
  {
    // Only for debuggin purposes. 
    std::cout<< "Joint " << i << " is " << plant.get_joint(multibody::JointIndex(i)).name() << std::endl;
    joint_names.push_back(plant.get_joint(multibody::JointIndex(i)).name());
  }
  assert(joint_names.size() == plant.num_actuators());

  // All the gains here - 
  Eigen::VectorXd kp(12);
  Eigen::VectorXd ki(12);
  Eigen::VectorXd kd(12);
  
  const MatrixX<double> B = plant.MakeActuationMatrix().transpose();
  auto B_ = B.block(6, 0, 12, 12);
  const auto state = plant.GetPositionsAndVelocities(*plant_context);
  
  auto controller = builder.AddSystem<drake::systems::controllers::PidController<double>>(state, B_, kp, ki, kd);
  // auto pid_controller = builder.AddSystem<systems::PidController<double>>(
  //   plant.num_actuated_dofs(), plant.num_actuated_dofs());
  constant_zero_source->set_name("Constant Zero Source");

  builder.Connect(plant.get_state_output_port(), controller->get_input_port());
  builder.Connect(controller->get_output_port(), plant.get_actuation_input_port());

  // Add the visualization.
  AddDefaultVisualization(&builder);

  auto diagram = builder.Build();
  drake::systems::Simulator<double> simulator(*diagram);
  
  // //Simulate
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