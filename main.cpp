#include <iostream>
#include <fstream>
#include <ctime>
#include <string>
#include <cmath>

#include <thread>
#include <chrono>
#include <atomic>

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

    int steps = 10;

    std::atomic<int> busy_p_mat;
    busy_p_mat = 0;
    std::atomic<bool> running;
    running = true;

    DPModel dp_model(steps, &busy_p_mat);
    std::cout << "creating a new DP model is done" << std::endl;
    int N = dp_model.N;
    int n_x = dp_model.x.n;
    int n_w = dp_model.w.n;
    int n_u = dp_model.u.n;

    std::cout << n_x << ", " << n_w << ", " << n_u << std::endl;

    float *value = new float[(N+1)*n_x*n_w]{};
    int *action = new int[N*n_x*n_w]{};

    int block_size = 64;
    if (block_size >= n_w)
        block_size = n_w/2;


    gpu_main(&dp_model, block_size, value, action, &busy_p_mat, &running);

    // std::thread t_dp(&DPModel::daemon, dp_model, &running);
    // std::thread t_gpu(gpu_main, &dp_model, block_size, value, action, &busy_p_mat, &running);

    // t_dp.join();
    // t_gpu.join();

    // result_to_file(&dp_model, "gpu", value, action);

    // if compared with CPU
    if (true)
    {
        CPUSolver cpu_solver(&dp_model, &busy_p_mat);
        start = std::clock();
        solver_type = "cpu";
        cpu_solver.solve();
        cpu_duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
        std::cout << std::endl << "CPU time: " << cpu_duration << " s" << std::endl;
        result_to_file(&dp_model, solver_type, cpu_solver.value, cpu_solver.action);

        //check error
        int error_flag = 0;
        float max_percent = 0;
        int max_per_idx = 0;
        float max_error = 0;
        float *value_error = new float[(N+1)*n_x*n_w]{};
        float *error_percent = new float[(N+1)*n_x*n_w]{};
        int *action_error = new int[N*n_x*n_w]{};
        int cnt = 0;
        for (int i = 0; i < (N+1)*n_x*n_w; ++i)
        {
            value_error[i] = fabs(value[i] - cpu_solver.value[i]);
            if (value_error[i] > max_error)
                max_error = value_error[i];

            error_percent[i] = value_error[i]/cpu_solver.value[i];
            if (error_percent[i] > max_percent)
            {
                max_percent = error_percent[i];
                max_per_idx = i;
            }
            
            if (error_percent[i] > 1e-6)
            {
                error_flag ++;
            }
            cnt ++;
        }
        std::cout << "max error percentage is: " << max_percent << std::endl;

        if (error_flag > 0)
        {
            std::cout << "value error found! " << error_flag << std::endl;
        }
        else
        {
            std::cout << "no value error was found" << std::endl;
        }

        // error_flag = 0;
        // for (int i = 0; i < N*n_x*n_w; ++i)
        // {
        //     action_error[i] = action[i] - cpu_solver.action[i];
        //     if (action_error[i] != 0)
        //     {
        //         error_flag ++;
        //     }
        // }
        // if (error_flag > 0)
        // {
        //     std::cout << "action error found! " << error_flag << std::endl;
        // }
        // else
        // {
        //     std::cout << "no action error was found" << std::endl;
        // }

        solver_type = "diff";
        // result_to_file(&dp_model, solver_type, value_error, action_error);
    }


    return 0;
}