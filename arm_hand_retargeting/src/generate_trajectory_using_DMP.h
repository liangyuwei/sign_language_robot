#include <iostream>
#include <math.h>


// For Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> // for Map()

// For file write and read
#include <string>
#include "H5Cpp.h"
//#include "H5Location.h"


using namespace std;
using namespace Eigen;
using namespace H5;



struct DMP_trajs
{
  // size is 3 x N
  MatrixXd y_lrw;
  MatrixXd y_lew;
  MatrixXd y_rew;
  MatrixXd y_rw; // right wrist
  
  MatrixXd y_re; // right elbow
  MatrixXd y_lw; // left wrist
  MatrixXd y_le; // left elbow

};


bool write_h5(const std::string file_name, const std::string group_name, const std::string dataset_name, const int ROW, const int COL, std::vector<std::vector<double>> data_vector);

std::vector<std::vector<double>> read_h5(const std::string file_name, const std::string group_name, const std::string dataset_name);

class DMPTrajectoryGenerator
{
  public:
    // read from h5 file when initializing
    DMPTrajectoryGenerator(std::string file_name, std::string group_name);
    ~DMPTrajectoryGenerator(){};

    
    // Variables
    unsigned int train_nbData = 400; // number of resampled path points used in learning DMP, which is also related to Yr data
    //unsigned int num_datapoints = 50; // expected number of output trajectory !!!
    double kP = 64;
    double kV = sqrt(2.0*kP); // spring and damping factors
    double dt = 0.01;

    MatrixXd sIn; // 400 x 1, share the same time profile
    
    MatrixXd Yr_lrw; // 3 x 400
    MatrixXd Yr_lew;
    MatrixXd Yr_rew;
    MatrixXd Yr_rw; // sum(psi*w)/sum(psi), which is later multiplied by x and (g-y0) modulation

    MatrixXd lrw_goal; // 1 x 3
    MatrixXd lrw_start;
    MatrixXd lew_goal;
    MatrixXd lew_start;
    MatrixXd rew_goal;
    MatrixXd rew_start;
    MatrixXd rw_goal;
    MatrixXd rw_start;  // original starts and goals
  
    std::string file_name, group_name;

    // Store orientation and glove angle data in here
    // size is 50 x DOF
    MatrixXd l_wrist_quat_traj, r_wrist_quat_traj, l_glove_angle_traj, r_glove_angle_traj; // 50 x DOF


    // convert to MatrixXd
    MatrixXd convert_to_matrixxd(std::vector<std::vector<double>> input);
    
    // debug
    bool save_eigen_matrix_h5(MatrixXd data, std::string name);
    void debug_repro();
    void debug_generalize();
    //void debug_print();
    //void debug_store(); // only used for debug, no need to store anything during run-time
    //void debug_test_noise();

  
    // interpolate to compute
    //void interpolate_hseq(MatrixXd pass_points);
    MatrixXd interpolate_trajectory(MatrixXd original_trajectory, unsigned int num_datapoints);

    
    // compute final output (f_seq, h_seq -> y_seq)
    //void compute_yseq();


    // Combining the pipeline
    //Matrix<double, >
    //MatrixXd generate_trajectories(MatrixXd lrw_new_goal); // input new goals and starts and return generalized trajectories
    MatrixXd generate_trajectory(MatrixXd new_goal, MatrixXd new_start, MatrixXd Yr, unsigned int num_datapoints);

    DMP_trajs generate_trajectories(MatrixXd lrw_new_goal, MatrixXd lrw_new_start, // column vectors
                                    MatrixXd lew_new_goal, MatrixXd lew_new_start,
                                    MatrixXd rew_new_goal, MatrixXd rew_new_start,
                                    MatrixXd rw_new_goal, MatrixXd rw_new_start,
                                    unsigned int num_datapoints);

};

