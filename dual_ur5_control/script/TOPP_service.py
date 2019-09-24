#!/usr/bin/env python

# ros
import rospy

# others
import string
from pylab import *
import numpy
import copy
import time

# TOPP
from TOPP import TOPPbindings
from TOPP import TOPPpy
from TOPP import Trajectory
from TOPP import Utilities


# service files
from raw_totg.srv import PathToTraj, PathToTrajResponse
from trajectory_msgs.msg import JointTrajectoryPoint


def call_TOPP(req):


  ### Get the request path and prepare for TOPP
  t0 = time.time()
  print("= Get the request path...")
  path_points = req.path
  vel_limits = req.vel_limits
  acc_limits = req.acc_limits
  discrtimestep = req.timestep
  

  # create a path going through viapoints
  print("== Create a path going through viapoints...")
  dim = len(path_points[0].positions)
  ndata = len(path_points)
  path_array = np.zeros((dim, ndata))
  for i in range(ndata):
    path_array[:, i] = np.array(path_points[i].positions)
  print("=== Interpolate viapoints...")
  #import pdb
  #pdb.set_trace()
  path = Utilities.InterpolateViapoints(path_array) # Interpolate using splines

  # Constraints
  vmax = np.array(vel_limits)
  amax = np.array(acc_limits)
  

  ### Set up the TOPP instance
  print("==== Set up a TOPP instance...")
  t1 = time.time()
  trajectorystring = str(path)
  uselegacy = True # True to use KinematicLimits(vel & acc)
  if uselegacy: #Using the legacy KinematicLimits (a bit faster but not fully supported)
    constraintstring = str(discrtimestep)
    constraintstring += "\n" + string.join([str(v) for v in vmax])
    constraintstring += "\n" + string.join([str(a) for a in amax])
    x = TOPPbindings.TOPPInstance(None,"KinematicLimits",constraintstring,trajectorystring);
  else: #Using the general QuadraticConstraints (fully supported)
    constraintstring = str(discrtimestep)
    constraintstring += "\n" + string.join([str(v) for v in vmax])
    constraintstring += TOPPpy.ComputeKinematicConstraints(path, amax, discrtimestep) 
    x = TOPPbindings.TOPPInstance(None,"QuadraticConstraints",constraintstring,trajectorystring);


  ### Run TOPP
  print("===== Run TOPP...")
  t2 = time.time()
  #import pdb
  #pdb.set_trace()
  ret = x.RunComputeProfiles(0,0)
  x.ReparameterizeTrajectory()


  ### Convert resultant array to plan
  print("====== Convert resultant array to plan...")
  t3 = time.time()
  x.WriteResultTrajectory() # be sure to write the result before reading it!!!
  traj = Trajectory.PiecewisePolynomialTrajectory.FromString(x.restrajectorystring)
  traj_point = JointTrajectoryPoint()
  traj_points = [] #PathToTrajResponse() # just an empty list
  t = 0.0
  while t < traj.duration:
    traj_point.positions = traj.Eval(t).tolist()
    traj_point.velocities = traj.Evald(t).tolist()
    traj_point.accelerations = traj.Evaldd(t).tolist()
    traj_point.time_from_start = rospy.Duration(t)
    traj_points.append(copy.deepcopy(traj_point))
    t += discrtimestep
  # the last point
  traj_point.positions = traj.Eval(traj.duration).tolist()
  traj_point.velocities = traj.Evald(traj.duration).tolist()
  traj_point.accelerations = traj.Evaldd(traj.duration).tolist()
  traj_point.time_from_start = rospy.Duration(traj.duration)
  traj_points.append(copy.deepcopy(traj_point))


  ### Display statistics
  t4 = time.time()
  print(">>>>> Statistics of TOPP <<<<<")
  print("Dimension of path point: " + str(path_array.shape[0]))
  print("Number of data points: " + str(path_array.shape[1]))
  print("Using legacy: " + str(uselegacy))
  print("Discretization step: " + str(discrtimestep) + " s")
  print("Obtain request path and get prepared: " + str(t1-t0) + " s")
  print("Setup TOPP: " + str(t2-t1) + " s")
  print("Run TOPP: " + str(t3-t2) + " s")
  print("Convert resultant array to plan: " + str(t4-t3) + " s")
  print("Total: " + str(t4-t0) + " s")
  print("Trajectory duration before TOPP: " + str(path.duration) + " s")
  print("Trajectory duration after TOPP: " + str(traj.duration) + " s")
  print(">>>>> End <<<<<")

  ### Return result
  res = PathToTrajResponse()
  res.traj = traj_points
  return res


def start_TOPP_server():

  rospy.init_node('TOPP_server')
  s = rospy.Service('TOPP_server', PathToTraj, call_TOPP)
  print("Ready to call TOPP.")
  rospy.spin()


if __name__ == '__main__':

  start_TOPP_server()


