^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sr_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.0 (2015-04-07)
------------------
* removing cartesian control - not implemented in ros-control + changing to the new traj controller
* Inertia now exact values, with inertia tensor at COM (gazebo and ode support the small values now)
  Removed explicit viscous damping from sr_gazebo_plugin,
  enabled springDamper in each joint (implicit viscous damping)
  changed simulation parameters for stability
  and retuned the controllers, explicitely define friction_deadband for simulated controllers (default is for real hand and is too high)
* Added mechanicalReduction to get valid URDF, still warnings about doubled joints or collision that are allowed actually
* Fixed color problems in biotac and forearm_lite
* adding the missing is_lite=0 param
* Rough weight guess for forearm
* Remove wrist mount collider
* Fix forearm collision.
* Fix forearm color in gazebo
* Fix scale
* Rotate the forearm
* Bring in the new forearm part.
* Start on xacro.
* Added a wrist mount collision box (motor and muscle hand)
  Added more precision to the palm collision shape (still using boxes)
* Add model for bimanual motor hand robot
* Changed inertia and force limits for biotac fingertip
* Upgraded kinect definition to use plugins, reactivated simulated kinect.
* Add the missing mesh for the finger biotac adapter.
* sr_description upgrade from groovy to hydro + update from hydro to indigo
  new collision model port from hydro to indigo
  fixed biotac finger missing the prefix parameter and middle transmission parameters
  fixed thumb vibrations (tuning and collision model)
  updated middle transmission to new J0Transmission syntax

1.3.1 (2014-07-18)
------------------

1.3.0 (2014-02-11)
------------------
* first hydro release
