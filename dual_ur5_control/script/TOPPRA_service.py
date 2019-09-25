#!/usr/bin/env python3
# call python3 to execute this script!!!

# ros
import rospy


# toppra
#import pyximport
#pyximport.install()
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo

# others
import matplotlib.pyplot as plt
import time
import numpy as np
import copy

# service files
from raw_totg.srv import PathToTraj, PathToTrajResponse
from trajectory_msgs.msg import JointTrajectoryPoint

# output python version
#import sys
#print("=== Python Version: " + sys.version)


def call_TOPPRA(req):

  ### Get the request path and prepare for TOPP-RA
  t0 = time.time()
  print("= Get the request path...")
  path_plan = req.path
  vel_limits = req.vel_limits
  acc_limits = req.acc_limits
  dt = req.timestep

  # kinematic constraints (and constraint object)
  vlim = np.vstack((-np.array(vel_limits), vel_limits)).T # list is not an operand type for unary
  alim = np.vstack((-np.array(acc_limits), acc_limits)).T
  pc_vel = constraint.JointVelocityConstraint(vlim)
  pc_acc = constraint.JointAccelerationConstraint(alim, discretization_scheme=constraint.DiscretizationType.Interpolation)

  # convert to waypoint array
  print("== Convert to an array of waypoints...")
  dof = len(path_plan[0].positions) 
  ndata = len(path_plan)
  path_array = np.zeros((ndata, dof))
  for i in range(ndata):
    path_array[i, :] = np.array(path_plan[i].positions)
  path = ta.SplineInterpolator(np.linspace(0, 1, ndata), path_array)


  ### Setup a TOPP-RA instance
  t1 = time.time()
  print("=== Set up a TOPP-RA instance and run the algorithm...")
  gridpoints = np.linspace(0, path.duration, 400) #3000) #2000) #1000) #800) #400) #200) # why grid points??? it doesn't seem to have anything to do with the result...
  instance = algo.TOPPRA([pc_vel, pc_acc], path, gridpoints=gridpoints, solver_wrapper='seidel')
  # different solver wrappers -- 'seidel'(fastest), 'hotqpoases'(second fastest), 'cvxpy'
  # 'seidel' can fail for ill-posed path parameterization instances
  t2 = time.time()
  jnt_traj, aux_traj, int_data = instance.compute_trajectory(0, 0, return_data=True)


  ### Run TOPP-RA to get the result
  t3 = time.time()
  print("==== Get the resultant pos, vel, acc and time profiles...")
  # get time sequence
  t = 0.0
  t_seq = []
  while t < jnt_traj.get_duration():
    t_seq.append(t)
    t += dt
  t_seq.append(jnt_traj.get_duration())

  # get positions, velocities and accelerations
  q_seq = jnt_traj.eval(t_seq)
  qd_seq = jnt_traj.evald(t_seq)
  qdd_seq = jnt_traj.evaldd(t_seq)

  # convert to plan and return
  print("===== Convert to plan and return...")
  traj_point = JointTrajectoryPoint()
  traj_points = [] #PathToTrajResponse() # just an empty list
  for i in range(len(t_seq)):
    traj_point.positions = q_seq[i, :].tolist()
    traj_point.velocities = qd_seq[i, :].tolist()
    traj_point.accelerations = qdd_seq[i, :].tolist()
    traj_point.time_from_start = rospy.Duration(t_seq[i])
    traj_points.append(copy.deepcopy(traj_point))
  t4 = time.time()


  ### Display statistics
  print(">>>>> Statistics of TOPP-RA <<<<<")
  print("Dimension of path point: " + str(dof))
  print("Number of data points: " + str(ndata))
  print("Obtain request path and get prepared: " + str(t1-t0) + " s")
  print("Setup TOPP-RA instance: " + str(t2-t1) + " s")
  print("Run TOPP-RA to compute the trajectory: " + str(t3-t2) + " s")
  print("Get the results and convert to plan: " + str(t4-t3) + " s")
  print("Total: " + str(t4-t0) + " s")
  print(">>>>> End <<<<<")


  ### Return result
  res = PathToTrajResponse()
  res.traj = traj_points
  return res


def start_TOPPRA_server():

  rospy.init_node('TOPPRA_server')
  s = rospy.Service('TOPPRA_server', PathToTraj, call_TOPPRA)
  print("Ready to call TOPP-RA")
  rospy.spin()


if __name__ == '__main__':
 
  start_TOPPRA_server()






