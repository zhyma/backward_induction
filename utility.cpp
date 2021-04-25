#include "utility.h"

#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include <sstream>

template<typename T>
int mat_to_file(std::string file_name, int dim_len, long *dim, T *mat)
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
    long x = 1;
    for(int i = 1; i < dim_len; ++i)
    {
        x = x * dim[i];
    }

    // usually the dim[0] is the time k
    for (int i = 0; i < k; ++i)
    {
        for (long j = 0; j < x; ++j)
        {
            long idx = i * x + j;
            out_file << mat[idx] << ",";
        }
        out_file << std::endl;
    }
    out_file.close();

    return 0;
}

template int mat_to_file(std::string file_name, int dim_len, long *dim, int *mat);
template int mat_to_file(std::string file_name, int dim_len, long *dim, float *mat);
template int mat_to_file(std::string file_name, int dim_len, long *dim, long *mat);
template int mat_to_file(std::string file_name, int dim_len, long *dim, unsigned long long int *mat);

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

DataLoader::DataLoader(std::string filename)
{
    if(FILE *file = fopen(filename.c_str(), "r"))
    {
        fclose(file);
    }
    else
    {
        std::cout << "ERROR! read file failed!" << std::endl;
        return;
    }
    f_name = filename;
    
    in_file.open(filename, std::ios::in);

    std::string line_str;
    std::stringstream ss_param;
    getline(in_file, line_str);
    ss_param.clear();
    ss_param.str(line_str); 
    for (int i = 0; i < 3; ++i)
    {
        getline(ss_param, line_str, ',');
        int pos = line_str.find('=');
        line_str.erase(0, pos+1);
        param[i] = stof(line_str);
    }

    bool end_of_file = false;
}

DataLoader::~DataLoader()
{
    in_file.close();
    std::cout << f_name << " closed" << std::endl;
}


int DataLoader::read_state(float &dc, int &intention)
{
    std::string line_str;
    std::stringstream ss_param;
    std::string arr[4];

    getline(in_file, line_str);
    if (in_file.eof())
    {
        end_of_file = true;
        return -1;
    }
    else if (line_str.find("end")!=std::string::npos)
    {
        return 0;
    }
    else
    {
        ss_param.clear();
        ss_param.str(line_str); 
        for (int j = 0; j < 4; ++j)
        {
            getline(ss_param, line_str, ',');
            arr[j] = line_str;
        }
        dc = stof(arr[0]);
        intention = stoi(arr[3]);
    }

    return true;
}

int DataLoader::next_trial()
{
    std::string line_str;

    getline(in_file, line_str);
    while( line_str.find("end")==std::string::npos )
    {
        getline(in_file, line_str);
    }

    return 0;
}

DataWriter::DataWriter(std::string filename)
{
    out_file.open(filename, std::ios::out);
    out_file << std::setiosflags(std::ios::fixed) << std::setprecision(2);
}

template<typename T>
int DataWriter::write(T data)
{
    out_file << data <<std::endl;
}
template int DataWriter::write(const char * data);
template int DataWriter::write(float data);

DataWriter::~DataWriter()
{
    // std::cout << "file handle released" << std::endl;
    out_file.close();
}