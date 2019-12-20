#include <string>
#include <iostream>
#include <vector>
#include "H5Cpp.h"

using namespace H5;

bool write_h5(const std::string file_name, const std::string group_name, const std::string dataset_name, const int ROW, const int COL, std::vector<std::vector<double>> data_vector)
{
  // Set up file name and dataset name
  const H5std_string FILE_NAME(file_name);
  const H5std_string GROUP_NAME(group_name);
  const H5std_string DATASET_NAME(dataset_name);

  // Convert 2-dim std::vector to 2-dim raw buffer(array)
  double data[ROW][COL];
  for (int j = 0; j < ROW; j++)
  {
    for (int i = 0; i < COL; i++)
    data[j][i] = data_vector[j][i];
  }

  try
  {
    // Create a file (must be an existing file)
    H5File file( FILE_NAME, H5F_ACC_RDWR );

    // Create a group (create and open?)
    Group group(file.createGroup(GROUP_NAME));

  
    // Set up datatype and dataspace for the dataset to be store
    hsize_t dimsf[2];              // dataset dimensions
    dimsf[0] = ROW;
    dimsf[1] = COL;
    DataSpace dataspace(2, dimsf);
    IntType datatype( PredType::NATIVE_DOUBLE );
    datatype.setOrder( H5T_ORDER_LE );


    // Way 1 - Create a dataset within a 'group'
    DataSet dataset1 = group.createDataSet(DATASET_NAME, datatype, dataspace);

    // Way 2 - Create a new dataset within the 'file'
    DataSet dataset2 = file.createDataSet( DATASET_NAME, datatype, dataspace );


    //Write the data to the dataset using default memory space, file space, and transfer properties.
    dataset1.write( data, PredType::NATIVE_DOUBLE );
    dataset2.write( data, PredType::NATIVE_DOUBLE );

  } // File and group will be closed as their instances go out of scope

  // catch failure caused by the H5File operations
  catch( FileIException error )
  {
    error.printErrorStack();
    return -1;
  }
  // catch failure caused by the DataSet operations
  catch( DataSetIException error )
  {
    error.printErrorStack();
    return -1;
  }
  // catch failure caused by the DataSpace operations
  catch( DataSpaceIException error )
  {
    error.printErrorStack();
    return -1;
  }
  // catch failure caused by the DataSpace operations
  catch( DataTypeIException error )
  {
    error.printErrorStack();
    return -1;
  }

  // Finish
  return true;

}


// Read h5 file for joint path
std::vector<std::vector<double>> read_h5(const std::string file_name, const std::string group_name, const std::string dataset_name)
{
  // Set up file name and dataset name
  const H5std_string FILE_NAME(file_name);
  const H5std_string DATASET_NAME(dataset_name);
  const H5std_string GROUP_NAME(group_name);

  try
  {
    // Open the specified file and the specified dataset in the file.
    H5File file( FILE_NAME, H5F_ACC_RDONLY );
    //DataSet dataset = file.openDataSet(DATASET_NAME)

    // Open a group 
    Group group = file.openGroup(GROUP_NAME);
    DataSet dataset = group.openDataSet(DATASET_NAME);

    // Get the class of the datatype that is used by the dataset.
    H5T_class_t type_class = dataset.getTypeClass();

    // Get dataspace of the dataset.
    DataSpace dataspace = dataset.getSpace();

    // Get the dimension size of each dimension in the dataspace and display them.
    hsize_t dims_out[2];
    int ndims = dataspace.getSimpleExtentDims( dims_out, NULL);
    int ROW = dims_out[0], COL = dims_out[1];

    // Read data into raw buffer(array) and convert to std::vector
    double data_array[ROW][COL];
    dataset.read(data_array, PredType::NATIVE_DOUBLE);
    std::vector<std::vector<double>> data_vector(ROW, std::vector<double>(COL));
    for (int j = 0; j < dims_out[0]; j++)
    {
      for (int i = 0; i < dims_out[1]; i++)
        data_vector[j][i] = data_array[j][i];
    }

    return data_vector;

  } 
   // catch failure caused by the H5File operations
   catch( FileIException error )
   {
      error.printErrorStack();
      exit(-1);
   }
   // catch failure caused by the DataSet operations
   catch( DataSetIException error )
   {
      error.printErrorStack();
      exit(-1);
   }
   // catch failure caused by the DataSpace operations
   catch( DataSpaceIException error )
   {
      error.printErrorStack();
      exit(-1);
   }
   // catch failure caused by the DataSpace operations
   catch( DataTypeIException error )
   {
      error.printErrorStack();
      exit(-1);
   }

}



int main (void)
{

  // Set up test array
  const int ROW = 5;                    // dataset dimensions
  const int COL = 6;
  std::vector<std::vector<double> > data_vector(ROW, std::vector<double>(COL));
  for (int j = 0; j < ROW; j++)
  {
    for (int i = 0; i < COL; i++)
      data_vector[j][i] = i + j;
  }
  std::cout << "The data to store in h5 file is: " << std::endl;
  for (int i = 0; i < ROW; ++i)
  {
    for (int j = 0; j < COL; ++j)
      std::cout << data_vector[i][j] << " ";
    std::cout << std::endl;
  }

  // Set up a multidimensional array
  const int HEIGHT = 4;
  const int WIDTH = 6;
  const int DEPTH = 2;
  std::vector<std::vector<std::vector<double> > > data_multi_array;
  for(int i = 0; i < DEPTH; ++i)
  {
    std::vector<std::vector<double> > tmp_vec(HEIGHT, std::vector<double>(WIDTH));
    for(int j = 0; j < HEIGHT; ++j)
    {
      for(int k = 0; k < WIDTH; ++k)
      {
        tmp_vec[j][k] = j + k;
      }
    }
    data_multi_array.push_back(tmp_vec);
  }
  // display  
  std::cout << "========== Display the 3-dim array ==========" << std::endl;
  for(int i = 0; i < DEPTH; ++i)
  {
    std::cout << "Depth " << i+1 << ":" << std::endl;
    for(int j = 0; j < HEIGHT; ++j)
    {
      for(int k = 0; k < WIDTH; ++k)
      {
        std::cout << data_multi_array[i][j][k] << " ";
      }
      std::cout << std::endl;
    }
  }


  // Test writing
  const std::string file_name = "test.h5";
  const std::string dataset_name = "TestDataSet";
  const std::string group_name = "TestGroup";
  write_h5(file_name, group_name, dataset_name, ROW, COL, data_vector);



  // Test reading
  std::vector<std::vector<double>> read_data_vector = read_h5(file_name, group_name, dataset_name);
  int R_ROW = read_data_vector.size();
  int R_COL = read_data_vector[0].size();
  std::cout << "The size of the data is: " << R_ROW << " x " << R_COL << std::endl;
  std::cout << "The data read from h5 file is: " << std::endl;
  for (int i = 0; i < R_ROW; ++i)
  {
    for (int j = 0; j < R_COL; ++j)
      std::cout << read_data_vector[i][j] << " ";
    std::cout << std::endl;
  }


  return 0;  // successfully terminated

}
