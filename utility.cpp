#include "utility.h"

#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>

template<typename T>
int mat_to_file(std::string file_name, int dim_len, int *dim, T *mat)
{
    std::ofstream out_file;
    out_file.open("output/" + file_name + ".csv", std::ios::out);
    out_file << std::setiosflags(std::ios::fixed) << std::setprecision(2);

    for (int i = 0; i < dim_len; ++i)
    {
        out_file << dim[i] << ",";
    }
    out_file << std::endl;

    int k = dim[0];
    int x = 1;
    for(int i = 1; i < dim_len; ++i)
    {
        x = x * dim[i];
    }

    // usually the dim[0] is the time k
    for (int i = 0; i < k; ++i)
    {
        for (int j = 0; j < x; ++j)
        {
            int idx = i * x + j;
            out_file << mat[idx] << ",";
        }
        out_file << std::endl;
    }
    out_file.close();

    return 0;
}

template int mat_to_file(std::string file_name, int dim_len, int *dim, int *mat);
template int mat_to_file(std::string file_name, int dim_len, int *dim, float *mat);
template int mat_to_file(std::string file_name, int dim_len, int *dim, unsigned long long *mat);

int result_to_file(std::string solver_type, int *dim, float *v, int * a)
{
    int N = dim[0];
    int n_x = dim[1];
    int n_w = dim[2];

    std::ofstream out_value;
    out_value.open("output/" + solver_type + "_value.csv", std::ios::out);
    out_value << std::setiosflags(std::ios::fixed) << std::setprecision(2);

    out_value << N << ",";
    out_value << n_x << ",";
    out_value << n_w << std::endl;
    std::cout << N << ", " << n_x << "," << n_w << std::endl;

    for (int k = 0; k <N+1; k++)
    {
        for (int xk = 0; xk < n_x; ++xk)
        {
            for (int wk = 0; wk < n_w; ++wk)
            {
                int idx = k*(n_x*n_w) + xk*n_w + wk;
                out_value << v[idx] << ",";
            }
        }
        out_value << std::endl;
    }
    out_value.close();

    std::ofstream out_action;
    out_action.open("output/" + solver_type + "_action.csv", std::ios::out);
    
    out_action << N << ",";
    out_action << n_x << ",";
    out_action << n_w << std::endl;

    for (int k = 0; k < N; k++)
    {
        for (int xk = 0; xk < n_x; ++xk)
        {
            for (int wk = 0; wk < n_w; ++wk)
            {
                int idx = k*(n_x*n_w) + xk*n_w + wk;
                out_action << a[idx] << ",";
            }
                
        }
        out_action << std::endl;
    }
    out_action.close();

    return 0;
}