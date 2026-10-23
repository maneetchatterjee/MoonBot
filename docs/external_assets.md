# External assets and licenses

MoonBot-X can be reproduced without downloading additional robot assets because the required upstream files are vendored and their revisions are pinned.

## Rover source

The rover visuals and suspension layout are derived from the Curio ROS simulation model of Roger Cheng's Sawppy rover:

- upstream repository: `https://github.com/srmainwaring/curio`
- pinned revision: `baec5d6d82454d7f4859479c7d481657b1890d09`
- license: BSD 3-Clause
- vendored files: chassis, differential, rocker, bogie, steering-assembly, and wheel STL meshes under `assets/meshes/sawppy/`
- license copy: `assets/licenses/curio-BSD-3-Clause.txt`

The visual meshes are uniformly scaled by 2x. The chassis ballast, collision primitives, masses, inertias, drive motors, steering actuators, joint constraints, and Panda mount are MoonBot-X simulation choices and are not Curio or stock Sawppy specifications.

## Manipulator source

The manipulator comes from the MuJoCo Menagerie Franka Emika Panda model:

- upstream repository: `https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_emika_panda`
- pinned Menagerie revision: `c96a32d28fb5da84da38c1da4d749e7a13212855`
- license: Apache License 2.0
- vendored source: `assets/mjcf/franka_emika_panda/`
- license copy: `assets/licenses/franka_emika_panda-Apache-2.0.txt`

The model generator imports the Panda link hierarchy, inertial tensors, joint limits, visual/collision assets, hand, finger equality, and gripper tendon without geometric scaling. MoonBot-X replaces the source joint position actuators with torque motors for the seven arm joints and adds project-specific sites and sensors. It should therefore be described as the Menagerie Panda rigid-body model with adapted actuation, not as an unmodified upstream scene or certified Franka hardware model.

## Non-selected references

NASA JPL's Open Source Rover and ESA's ExoMy were evaluated as alternatives. The former did not provide an immediately usable simulation mesh/URDF package in the inspected repository, while ExoMy's model is GPL-3.0. Neither project contributes files to the canonical MoonBot-X model.
