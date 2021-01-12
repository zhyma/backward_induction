#ifndef UTILITY_H_
#define UTILITY_H_

#include <string>
#include <fstream>
#include <iomanip>
#include "dp_model.h"

int mat_to_file(std::string file_name, int *dim, float *mat)
{
    std::ofstream out_file;
    out_file.open("output/" + file_name + ".csv", std::ios::out);
    out_file << std::setiosflags(std::ios::fixed) << std::setprecision(2);

    for (int i = 0; i < dim[0]; ++i)
    {
        for (int j = 0; j < dim[1]; ++j)
        {
            int idx = i * dim[1] + j;
            out_file << mat[idx] << ",";
        }
        out_file << std::endl;
    }
    out_file.close();

    return 0;
}

int result_to_file(DPModel * model, std::string solver_type, float *v, int * a)
{
    int N = model->N;
    int n_x = model->x.n;
    int n_w = model->w.n;
    int n_u = model->u.n;

    std::ofstream out_value;
    out_value.open("output/" + solver_type + "_value.csv", std::ios::out);
    out_value << std::setiosflags(std::ios::fixed) << std::setprecision(2);

    //title needs to be re-assigned
    for (int i = 0; i < model->x.n*model->w.n; ++i)
    {
        // xw_idx = xk*w_cnt + wk
        out_value << model->x.list[i/n_w] << ";";
        out_value << model->x.list[i%n_w] << ",";
    }
    out_value << std::endl;

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
    out_action << std::setiosflags(std::ios::fixed) << std::setprecision(2);
    for (int i = 0; i < model->x.n*model->w.n; ++i)
    {
        // xw_idx = xk*w_cnt + wk
        out_action << model->x.list[i/n_w] << ";";
        out_action << model->x.list[i%n_w] << ",";
    }
    out_action << std::endl;
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

int search_files(std::vector<std::string> *files, std::string search_key)
{
	DIR *dpdf;
    struct dirent *epdf;
    int cnt = 0;

    dpdf = opendir("output");
    if (dpdf != NULL){
        while (epdf = readdir(dpdf))
        {
            std::string name = epdf->d_name;
            std::size_t found = name.find(search_key);
            if (found != std::string::npos)
            {
                files->push_back(name);
                ++cnt;
            }
        }
    }
    closedir(dpdf);
    return cnt;
}

#endif //UTILITY_H_