#include <string>
#include <iostream>
#include <vector>
#include "H5Cpp.h"

using namespace H5;

bool write_h5(const std::string file_name, const std::string dataset_name, const int ROW, const int COL, std::vector<std::vector<double>> data_vector)
{
  // Set up file name and dataset name
  const H5std_string  FILE_NAME(file_name);
  const H5std_string  DATASET_NAME(dataset_name);

  // Convert 2-dim std::vector to 2-dim raw buffer(array)
  double data[ROW][COL];
  for (int j = 0; j < ROW; j++)
  {
    for (int i = 0; i < COL; i++)
    data[j][i] = data_vector[j][i];
  }

  try
  {
    // Create a file, or earse all data of an existing file
    H5File file( FILE_NAME, H5F_ACC_TRUNC );
      
    // Create data space for fixed size dataset
    hsize_t dimsf[2];              // dataset dimensions
    dimsf[0] = ROW;
    dimsf[1] = COL;
    DataSpace dataspace(2, dimsf);

    // Define datatype for the data in the file.
    IntType datatype( PredType::NATIVE_DOUBLE );
    datatype.setOrder( H5T_ORDER_LE );

    // Create a new dataset within the file using defined dataspace and datatype
    DataSet dataset = file.createDataSet( DATASET_NAME, datatype, dataspace );

    //Write the data to the dataset using default memory space, file space, and transfer properties.
    dataset.write( data, PredType::NATIVE_DOUBLE );

  } 
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
std::vector<std::vector<double>> read_h5(const std::string file_name, const std::string dataset_name)
{
  // Set up file name and dataset name
  const H5std_string  FILE_NAME(file_name);
  const H5std_string  DATASET_NAME(dataset_name);

  try
  {
    // Open the specified file and the specified dataset in the file.
    H5File file( FILE_NAME, H5F_ACC_RDONLY );
    DataSet dataset = file.openDataSet(DATASET_NAME);

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


  // Test writing
  const std::string file_name = "test.h5";
  const std::string dataset_name = "TestArray";
  write_h5(file_name, dataset_name, ROW, COL, data_vector);



  // Test reading
  std::vector<std::vector<double>> read_data_vector = read_h5(file_name, dataset_name);
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
