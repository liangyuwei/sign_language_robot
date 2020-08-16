#include "collision_checking.h"
#include <iostream>



int main(int argc, char** argv)
{

  std::vector<double> q_fengren_start = {-0.45356114,  0.03423116, -0.21550286, -0.18166481,  0.05833756,
        0.42442878, -0.16504643, -0.13673801,  0.10574684,  0.04755113,
        0.401288  ,  0.07277119, -0.04641443, -0.03134606, -0.02950217,
       -0.18448639, -0.02950217, -0.29183667, -0.04636056, -0.03134606,
       -0.21512006,  0.13832016, -0.08321451, -0.07855118, -0.02605913,
       -0.03225688, -0.17943979, -0.20613834, -0.12105841, -0.22580741,
       -0.12401753, -0.16770475, -0.07902519,  0.04772362, -0.03999095,
       -0.10017621};


  // Settings
  std::string urdf_file_name = "/home/liangyuwei/sign_language_robot_ws/src/ur_description/urdf/ur5_robot_with_hands.urdf";
    std::string srdf_file_name = "/home/liangyuwei/sign_language_robot_ws/src/sign_language_robot_moveit_config/config/ur5.srdf";


  // Set a class
  std::chrono::steady_clock::time_point time_start1 = std::chrono::steady_clock::now();

  std::ifstream urdf_file(urdf_file_name);
  std::ifstream srdf_file(srdf_file_name);
  std::stringstream urdf_string, srdf_string;
  urdf_string << urdf_file.rdbuf();
  srdf_string << srdf_file.rdbuf();

  std::chrono::steady_clock::time_point time_end1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used1 = std::chrono::duration_cast<std::chrono::duration<double>>(time_end1 - time_start1);
  std::cout << "time used for reading URDF and SRDF files:" << time_used1.count() << " s" << std::endl;



  //std::vector<double> min_dist_results;
  double min_dist_results;

  std::chrono::steady_clock::time_point time_start2 = std::chrono::steady_clock::now();

  ros::init(argc, argv, "test_collision_checking");

  std::chrono::steady_clock::time_point time_end2 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used2 = std::chrono::duration_cast<std::chrono::duration<double>>(time_end2 - time_start2);
  std::cout << "time used for initializing ROS node:" << time_used2.count() << " s" << std::endl;


  DualArmDualHandCollision dual_arm_dual_hand_collision(urdf_string.str(), srdf_string.str());


  min_dist_results = dual_arm_dual_hand_collision.check_collision(q_fengren_start);


  std::cout << "The result is " << min_dist_results << std::endl;

  //ros::spin();
  //spinner.stop();

  // Shut down
  //ros::shutdown();
  return 0;


}
