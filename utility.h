#ifndef UTILITY_H_
#define UTILITY_H_

#include <string>
#include <fstream>
#include <iomanip>

template<typename T>
int mat_to_file(std::string file_name, int dim_len, int *dim, T *mat);

int result_to_file(std::string solver_type, int *dim, float *v, int * a);

#endif //UTILITY_H_