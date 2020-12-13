#include <iostream>
#include <fstream>
#include <ctime>
#include <string>
#include <cmath>

#include "phy_model.h"
#include "dp_model.h"
#include "cpu_solver.h"
#include "gpu_solver.h"
#include "utility.h"

int main()
{
    std::clock_t start;
    double cpu_duration = 0;
    double gpu_duration = 0;
    std::string solver_type;

    PHYModel phy_model;
    int steps = 10;
    int n_x = 128;
    int n_w = 128;
    int n_u = 32;

    int block_size = 32;
    if (block_size >= n_w)
        block_size = n_w/2;

    DPModel dp_model(&phy_model, steps, n_x, n_w, n_u);
    std::cout << "creating a new DP model is done" << std::endl;
    int N = dp_model.N;

    std::string save_prob_mat = "p_mat";
    int dim[2] = {dp_model.w_set.count, dp_model.w_set.count};
    mat_to_file(save_prob_mat, dim, dp_model.prob_table);

    // float *value = new float[(N+1)*n_x*n_w]{};
    // int *action = new int[N*n_x*n_w]{};

    // int iter = 21;// default: 50
    // float avg = 0;
    // for(int i = 0; i < iter; ++i)
    // {
    //     start = std::clock();
    //     solver_type = "gpu";
    //     gpu_main(&dp_model, block_size, value, action);
    //     gpu_duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    //     // std::cout << "GPU time: " << gpu_duration << " s" << std::endl;
    //     std::cout << gpu_duration << ",";
    //     if(i > 0)
    //         avg += gpu_duration;
    //     // result_to_file(&dp_model, solver_type, value, action);
    // }
    // std::cout << std::endl << "gpu_duration: " << avg/(iter-1) << std::endl;

    // // if compared with CPU
    // if (true)
    // {
    //     start = std::clock();
    //     solver_type = "cpu";
    //     CPUSolver cpu_solver(&dp_model);
    //     cpu_duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    //     std::cout << std::endl << "CPU time: " << cpu_duration << " s" << std::endl;
    //     // result_to_file(&dp_model, solver_type, cpu_solver.value, cpu_solver.action);

    //     int error_flag = 0;
    //     for (int i = 0; i < (N+1)*n_x*n_w; ++i)
    //     {
    //         value[i] = abs(value[i] - cpu_solver.value[i]);
    //         if (value[i] > 0.0001)
    //         {
    //             error_flag ++;
    //         }
    //     }
    //     if (error_flag > 0)
    //     {
    //         std::cout << "value error found! " << error_flag << std::endl;
    //     }
    //     else
    //     {
    //         std::cout << "no error was found" << std::endl;
    //     }

    //     for (int i = 0; i < N*n_x*n_w; ++i)
    //         action[i] = action[i] - cpu_solver.action[i];

    //     // solver_type = "diff";
    //     // result_to_file(&dp_model, solver_type, value, action);
    // }


    return 0;
}