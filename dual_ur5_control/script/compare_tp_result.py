#!/usr/bin/env python


import sys
import copy
import rospy
import math
import tf
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list 
import h5py
import matplotlib.pyplot as plt

def main():


  try:
  
    print "============ Loading h5py file ..."
    import pdb
    pdb.set_trace()
    f = h5py.File("dual_ur5_joint_trajectory_DIFF_TIME_PARAMETERIZATION.h5", "r")

    
    print "============ Read data ..."
    # set target info
    index = "8"
    # 1 for TP, 2 for spline, 3 for TOTG, 4 for TOTG(Add TP first, ORDER matters), 5 for no TP
    # 6 for TOTG with nonzero(limited) acc(0.0 means unlimited) -- acc 2.0
    # 7 for TP -- acc 2.0
    # 8 for TOTG -- acc 10.0
    # 9 for TP -- acc 10.0
    left_or_right = "l"
    group_name = "traj_pair_" + left_or_right + "_" + index
    # read data
    pos = f[group_name]["pos"][:]
    vel = f[group_name]["vel"][:]
    acc = f[group_name]["acc"][:]
    time_from_start = f[group_name]["time_from_start"][:]
    # close file
    f.close()
    

    print "============ Plot the data"
    fig, axes = plt.subplots(nrows=3, ncols=1) # create a figure object
    axes[0, 0].set(title="Position of " + group_name)

      # obtain trajectory points
      tmp_points = np.ndarray((len(tmp_plan.joint_trajectory.points), 6), dtype=float)
      tmp_time = np.ndarray(len(tmp_plan.joint_trajectory.points))
      for n in range(len(tmp_plan.joint_trajectory.points)):
        tmp_points[n, :] = tmp_plan.joint_trajectory.points[n].positions
        tmp_time[n] = tmp_plan.joint_trajectory.points[n].time_from_start.to_sec()

      # plot the acquired data
      for ic in range(6):
        # set title
        axes[ir, ic].set(title=tmp_plan.joint_trajectory.joint_names[ic])
        # plot data
        #axes[ir, ic].plot(np.linspace(0, 10, len(tmp_plan.joint_trajectory.points)), tmp_points[:, ic])
        axes[ir, ic].plot(tmp_time, tmp_points[:, ic])

    # display
    plt.show()

    ''


    ### Use h5py to store the generated motion plan
    ''
    

    ''

    ### Open grippers
    '''
    print "============ Open grippers for Gazebo..."
    import pdb
    pdb.set_trace()
    tutorial.gripper_control(True, True)
    tutorial.gripper_control(True, False)
    '''

    ### Detach mesh
    ''
    print "============ Detach flash models..."
    import pdb
    pdb.set_trace()
    tutorial.scene.remove_attached_object("right_ee_link", fb_mesh_name)
    tutorial.scene.remove_attached_object("left_ee_link", fh_mesh_name)
    ''

    ### Remove mesh
    ''
    print "============ Remove flash models from the scene ..."
    import pdb
    pdb.set_trace()
    tutorial.remove_object(fb_mesh_name, timeout=4)
    tutorial.remove_object(fh_mesh_name, timeout=4)
    ''


  except rospy.ROSInterruptException:
    return

  except KeyboardInterrupt:
    return


if __name__ == '__main__':

  main()







