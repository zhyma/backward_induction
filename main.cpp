#include <iostream>

#include <string>
#include <cstring>

#include "simulate.h"
#include "front_car.h"
#include "dp_model.h"

int main(int argc, char *argv[])
{
    std::string modeStr = "";
    if (argc > 1)
    {
        modeStr.assign(argv[1], strlen(argv[1]));
    }
    else
        modeStr = "compare";

    if (modeStr == "one_step" || modeStr == "n_step"|| modeStr == "compare")
    {
        int pred_steps = 10;
        // int run_steps = 10;
        int run_steps = 10;

        DPModel dp_model(pred_steps, run_steps);
        if (modeStr == "compare")
        {
            std::cout << "run solver comparison for one step" << std::endl;
            one_iter(COMPARE, true, &dp_model);
        }
        else if (modeStr == "one_step")
        {
            // run one step
            std::cout << "run simulation for one step" << std::endl;
            if (argc > 2)
            {
                std::string solverStr = "";
                solverStr.assign(argv[2], strlen(argv[2]));
                if (solverStr == "cpu")
                    one_iter(CPU_SOLVER, true, &dp_model);
                else if(solverStr == "gpu")
                    one_iter(GPU_SOLVER, true, &dp_model);
            }
            else
                one_iter(CPU_SOLVER, true, &dp_model);
        }
        else if (modeStr == "n_step")
        { 
            // whole simulation
            std::cout << "run simulation for n steps" << std::endl;
            int pred_steps = 10;
            // int run_steps = 10;
            int run_steps = 10;

            DPModel dp_model(pred_steps, run_steps);
            run_iters(run_steps, GPU_SOLVER, &dp_model);
        }
    }
    
    else if (modeStr == "gen_data")
    {
        // generate front car data
        std::cout << "generate front car driving data" << std::endl;
        int iter = 10;
        if (argc > 2)
        {
            std::string val(argv[2]); 
            iter = std::stoi(val);
            if (iter <= 0)
                iter = 10;
        }
        std::cout << "Simulate the front car for " << iter << " times" << std::endl;
        
        fc_n_step_sim(iter);
    }

    std::cout << "done" << std::endl;

    return 0;
}