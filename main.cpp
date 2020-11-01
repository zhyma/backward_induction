#include <iostream>
#include <stdlib.h>
#include "phy_model.h"
#include "dp_model.h"
#include "gpu_solver.h"

int main()
{
    PHYModel phy_model;
    DPModel dp_model(&phy_model, 1000);
    gpu_main(&dp_model);

    return 0;
}