# HexaPod Walking Simulation in Drake
This project will be building a controller for the Trossen Robotics Hexapod robot (PhantomX). The original open source URDF and collision are taken from [HumaRobotics repo](https://github.com/HumaRobotics/phantomx_description). It had to be modified to make it work with Drake. 

![Phantom X model](/media/phantomx.png?raw=true "Phantom X model with the HumaRobotics meshes")


Note -> Just an FYI - Some inertial values didn't make sense from a Spatial Inertial standpoint, particularly, the triangle inequality. I changed them slightly to make everything work. Since the values are extremely small, I am positive that it won't affect the dynamics of the robot too much. 


## Bazel Project with Drake as an External
These running instructions and some starter code taken as is from [here](https://github.com/RobotLocomotion/drake-external-examples)

This pulls in Drake via the Bazel workspace mechanism.

For an introduction to Bazel, refer to
[Getting Started with Bazel](https://docs.bazel.build/versions/master/getting-started.html).

## Instructions

First, install the required Ubuntu packages:

```
sudo ../scripts/setup/linux/ubuntu/focal/install_prereqs
```
For Mac - 
```
../scripts/setup/mac/install_prereqs
```

Then, to build and test all apps:
```
bazel test //...
```

As an example to run a binary directly:
```
bazel run //apps:hex_simulation
```

You may also run the binary directly per the `bazel-bin/...` path that the
above command prints out; however, be aware that your working directories may
cause differences.  This is important when using tools like
`drake::FindResource` / `pydrake.common.FindResource`.
You may generally want to stick to using `bazel run` when able.

