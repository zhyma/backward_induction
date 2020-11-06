#include <iostream>
// #include <stdlib.h>
#include <fstream>
#include <ctime>
#include <string>
#include <cmath>

#include "phy_model.h"
#include "dp_model.h"
#include "gpu_solver.h"
#include "cpu_solver.h"
#include "utility.h"

int main()
{
    std::clock_t start;
    double gpu_duration = 0;
    std::string solver_type;

    PHYModel phy_model;
    DPModel dp_model(&phy_model, 3);
    std::cout << "creating a new DP model is done" << std::endl;
    int N = dp_model.N;
    int n_x = dp_model.x_set.count;
    int n_w = dp_model.w_set.count;
    int n_u = dp_model.u_set.count;

    float *value = new float[(N+1)*n_x*n_w]{};
    int *action = new int[N*n_x*n_w]{};
    float *q_table = new float[N*n_x*n_w*n_u]{};

    start = std::clock();
    solver_type = "cpu";
    CPUSolver cpu_solver(&dp_model);
    gpu_duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    std::cout << "CPU time: " << gpu_duration << " s" << std::endl;
    write_to_file(&dp_model, solver_type, cpu_solver.value, cpu_solver.action, cpu_solver.q_table);

    start = std::clock();
    solver_type = "gpu";
    gpu_main(&dp_model, value, action, q_table);
    gpu_duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    std::cout << "GPU time: " << gpu_duration << " s" << std::endl;
    write_to_file(&dp_model, solver_type, value, action, q_table);

    for (int i = 0; i < (N+1)*n_x*n_w; ++i)
        value[i] = abs(value[i] - cpu_solver.value[i])<0.01 ? 0:(value[i] - cpu_solver.value[i]);

    for (int i = 0; i < N*n_x*n_w; ++i)
        action[i] = action[i] - cpu_solver.action[i];

    for (int i = 0; i < N*n_x*n_w*n_u; ++i)
        q_table[i] = abs(q_table[i] - cpu_solver.q_table[i]) < 0.01?0:(q_table[i] - cpu_solver.q_table[i]);
    solver_type = "diff";
    write_to_file(&dp_model, solver_type, value, action, q_table);

    return 0;
}