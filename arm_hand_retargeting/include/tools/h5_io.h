#ifndef H5_IO_H
#define H5_IO_H

// HDF5 related
#include "H5Cpp.h"
//#include "H5Location.h"

// Common
#include <vector>
#include <string>
#include <iostream>

using namespace H5;

/// Write 3-dim array to h5 file
bool write_h5_3d(const std::string file_name, const std::string group_name, const std::string dataset_name, const int LAYER, const int ROW, const int COL, std::vector<std::vector<std::vector<double> > > data_vector);

/// Write 2-dim matrix to h5 file
bool write_h5(const std::string file_name, const std::string group_name, const std::string dataset_name, const int ROW, const int COL, std::vector<std::vector<double>> data_vector);

/// Read 2-dim matrix from h5 file
std::vector<std::vector<double>> read_h5(const std::string file_name, const std::string group_name, const std::string dataset_name);


#endif