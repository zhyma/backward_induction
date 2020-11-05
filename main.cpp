#include <iostream>
#include <stdlib.h>
#include "phy_model.h"
#include "dp_model.h"
#include "gpu_solver.h"
#include "cpu_solver.h"
#include "utility.h"

#include <iostream>
#include <fstream>
#include <ctime>

int main()
{
    std::clock_t start;
    double gpu_duration = 0;

    PHYModel phy_model;
    DPModel dp_model(&phy_model, 10);
    std::cout << "creating a new DP model is done" << std::endl;
    int N = dp_model.N;
    int n_x = dp_model.x_set.count;
    int n_w = dp_model.w_set.count;
    int n_u = dp_model.u_set.count;
    float *v = new float[(N+1)*n_x*n_w]{};
    // You may only need 
    int *a = new int[N*n_x*n_w]{};
    start = std::clock();
    gpu_main(&dp_model, v, a);
    gpu_duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    std::cout << "GPU time: " << gpu_duration << " s" << std::endl;
    write_to_file(&dp_model, v, a);

    return 0;
}