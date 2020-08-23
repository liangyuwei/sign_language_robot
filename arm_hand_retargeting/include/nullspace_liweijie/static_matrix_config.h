#ifndef _STATIC_MATRIX_CONFIG_H
#define _STATIC_MATRIX_CONFIG_H
#include "config_nullspace.h"
#include "transform_matrix_helper.h"

// namespace cfg{
    // Create static rotation matrices and translation vectors
        const static Eigen::Matrix<double,3,3>  Rl1  = Rrpy(double(rot_l1[0]),double(rot_l1[1]),double(rot_l1[2]));  const static Eigen::Matrix<double,3,1> Tl1  = create_translation_vector(double(trans_l1[0]),double(trans_l1[1]),double(trans_l1[2]));
        const static Eigen::Matrix<double,3,3>  Rl2  = Rrpy(double(rot_l2[0]),double(rot_l2[1]),double(rot_l2[2]));  const static Eigen::Matrix<double,3,1> Tl2  = create_translation_vector(double(trans_l2[0]),double(trans_l2[1]),double(trans_l2[2]));
        const static Eigen::Matrix<double,3,3>  Rl3  = Rrpy(double(rot_l3[0]),double(rot_l3[1]),double(rot_l3[2]));  const static Eigen::Matrix<double,3,1> Tl3  = create_translation_vector(double(trans_l3[0]),double(trans_l3[1]),double(trans_l3[2]));
        const static Eigen::Matrix<double,3,3>  Rl4  = Rrpy(double(rot_l4[0]),double(rot_l4[1]),double(rot_l4[2]));  const static Eigen::Matrix<double,3,1> Tl4  = create_translation_vector(double(trans_l4[0]),double(trans_l4[1]),double(trans_l4[2]));
        const static Eigen::Matrix<double,3,3>  Rl5  = Rrpy(double(rot_l5[0]),double(rot_l5[1]),double(rot_l5[2]));  const static Eigen::Matrix<double,3,1> Tl5  = create_translation_vector(double(trans_l5[0]),double(trans_l5[1]),double(trans_l5[2]));
        const static Eigen::Matrix<double,3,3>  Rl6  = Rrpy(double(rot_l6[0]),double(rot_l6[1]),double(rot_l6[2]));  const static Eigen::Matrix<double,3,1> Tl6  = create_translation_vector(double(trans_l6[0]),double(trans_l6[1]),double(trans_l6[2]));
        const static Eigen::Matrix<double,3,3>  Rl7  = Rrpy(double(rot_l7[0]),double(rot_l7[1]),double(rot_l7[2]));  const static Eigen::Matrix<double,3,1> Tl7  = create_translation_vector(double(trans_l7[0]),double(trans_l7[1]),double(trans_l7[2]));
        const static Eigen::Matrix<double,3,3>  Rr1  = Rrpy(double(rot_r1[0]),double(rot_r1[1]),double(rot_r1[2]));  const static Eigen::Matrix<double,3,1> Tr1  = create_translation_vector(double(trans_r1[0]),double(trans_r1[1]),double(trans_r1[2]));
        const static Eigen::Matrix<double,3,3>  Rr2  = Rrpy(double(rot_r2[0]),double(rot_r2[1]),double(rot_r2[2]));  const static Eigen::Matrix<double,3,1> Tr2  = create_translation_vector(double(trans_r2[0]),double(trans_r2[1]),double(trans_r2[2]));
        const static Eigen::Matrix<double,3,3>  Rr3  = Rrpy(double(rot_r3[0]),double(rot_r3[1]),double(rot_r3[2]));  const static Eigen::Matrix<double,3,1> Tr3  = create_translation_vector(double(trans_r3[0]),double(trans_r3[1]),double(trans_r3[2]));
        const static Eigen::Matrix<double,3,3>  Rr4  = Rrpy(double(rot_r4[0]),double(rot_r4[1]),double(rot_r4[2]));  const static Eigen::Matrix<double,3,1> Tr4  = create_translation_vector(double(trans_r4[0]),double(trans_r4[1]),double(trans_r4[2]));
        const static Eigen::Matrix<double,3,3>  Rr5  = Rrpy(double(rot_r5[0]),double(rot_r5[1]),double(rot_r5[2]));  const static Eigen::Matrix<double,3,1> Tr5  = create_translation_vector(double(trans_r5[0]),double(trans_r5[1]),double(trans_r5[2]));
        const static Eigen::Matrix<double,3,3>  Rr6  = Rrpy(double(rot_r6[0]),double(rot_r6[1]),double(rot_r6[2]));  const static Eigen::Matrix<double,3,1> Tr6  = create_translation_vector(double(trans_r6[0]),double(trans_r6[1]),double(trans_r6[2]));
        const static Eigen::Matrix<double,3,3>  Rr7  = Rrpy(double(rot_r7[0]),double(rot_r7[1]),double(rot_r7[2]));  const static Eigen::Matrix<double,3,1> Tr7  = create_translation_vector(double(trans_r7[0]),double(trans_r7[1]),double(trans_r7[2]));
// }

#endif