#include <iostream>

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

bool write_h5(const std::string file_name, const std::string group_name, const std::string dataset_name, const int ROW, const int COL, std::vector<std::vector<double>> data_vector);

std::vector<std::vector<double>> read_h5(const std::string file_name, const std::string group_name, const std::string dataset_name);

class TrajectoryGenerator
{
  public:
    // read from h5 file when initializing
    TrajectoryGenerator(std::string file_name, std::string group_name);
    ~TrajectoryGenerator(){};

    
    // Variables
    MatrixXd f_seq;  // N x 48 size !!!
    MatrixXd kp_list;
    MatrixXd pos_and_glove_id;
    MatrixXd quat_id;
    MatrixXd pass_time;
    MatrixXd pass_points; // not needed for adaptation; N x 48(dof)
    MatrixXd time_range;
    
    std::string file_name, group_name;

    MatrixXd h_seq;
    MatrixXd y_seq;

    // convert to MatrixXd
    MatrixXd convert_to_matrixxd(std::vector<std::vector<double>> input);


    // debug
    void debug_print();
    void debug_store(); // only used for debug, no need to store anything during run-time
    void debug_test_noise();

    // normalize quaternion part of data (g2o might break the unit quaternion constraint)
    MatrixXd normalize_quaternion_part(MatrixXd pass_points);

    // interpolate to compute
    void interpolate_hseq(MatrixXd pass_points);

    
    // compute final output (f_seq, h_seq -> y_seq)
    void compute_yseq();


    // Combining the pipeline
    MatrixXd generate_trajectory_from_passpoints(MatrixXd pass_points);


};

