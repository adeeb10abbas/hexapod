# Quadruped Walking Simulation in Drake
This project will be building a controller for the Unitree A1 Quadruped. 

![Picture of the A1 Quadruped
](media/a1.png)


### Running Instructions
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
bazel run //apps:quad_simulation
```

You may also run the binary directly per the `bazel-bin/...` path that the
above command prints out; however, be aware that your working directories may
cause differences.  This is important when using tools like
`drake::FindResource` / `pydrake.common.FindResource`.
You may generally want to stick to using `bazel run` when able.

