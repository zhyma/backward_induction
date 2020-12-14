- Test under Ubuntu 18.04TLS + CUDA v11.1.74.
<br />

- There are three main parts in this repo:
    1. A very simple linear model (phy_model.cpp/phy_model.h)
    2. A module to generate model for dynamic programming (dp_model.cpp/dp_model.h)
    3. Two module to solve the stochastic dynamic programming (cpu_solver.cpp/gpu_solver.cu)
<br />

- The dp_model search from time step k=0 to k=N. By giving the constraint of x and the granularity, discritizing x into states and search for all possible states. This module will generate a <S, A, R(cost-to-go), P, S'> and transition probability matrix.
<br />

- The probability model will be saved in "prob.csv"
- The value table is saved to "value.csv"
- The optimal action is saved to "action.csv"
- Use "show_value.py" to visualize the value table and the optimal actions.