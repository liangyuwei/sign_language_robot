This package contains the description for Shadow Robot's Hand and Arm, as well as some additional models used in our robot (kinect, etc...).

You can find the different complete models in [robots](robots).

You can also find more information about the computation of the [arm kinematics](doc/ArmInertia.md) and of the [hand kinematics](doc/HandInertia.md).


*Tips: To automatically reformat the different xacro files you can use the following command:*

```bash
find . -name "*.urdf.xacro" -type f -exec tidy -xml -i -m -q "{}" \;
```
