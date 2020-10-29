#include <iostream>
#include <stdlib.h>
#include "phy_constraint.h"
#include "dp_model.h"
#include "gpu_solver.h"

int main()
{
    DPModel dp_model;
    gpu_main(&dp_model);
    float x_ = linear_model_cpu(0, 0, 0);
    return 0;
}