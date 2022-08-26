- Supplementary code for paper [Using GPU to Accelerate Backward Induction for Vehicle Speed Optimal Control](https://www.sae.org/publications/technical-papers/content/2022-01-0089/)
- Test under Ubuntu 18.04TLS + CUDA v11.1.74.

- There are three main parts in this repo:
    1. A simulator to config how to solve the problem.
    2. A Dynamic programming model that: 
        1. Provide a physical model and constraint of the vehicle
        2. Discretizing continuous distance, velocity, and acceleration into states, and generate state transition
    3. Two module to solve the stochastic dynamic programming (cpu_solver.cpp/gpu_solver.cu)

- Usage:
    - Place the "config.xml" under your executing directory.
        E.g.: Suppose you are at "bi_dir/", the executable is "bi_dir/build/dp_solver". Place the "config.xml" under "bi_dir/", then run "bi_dir/build/dp_solver" with parameters. If you are under "bi_dir/build/dp_solver", then place the "config.xml" under "bi_dir/build/", and run ./dp_solver" with parameters.
    - Currently could only do one-time prediction. The command with parameters are:
        - "./build/dp_solver one_step cpu": solve the problem once by using CPU.
        - "./build/dp_solver one_step gpu": solve the problem once by using GPU.

- Historical front car behavior data should placed under "output" folder, with a prefix of "front_car_data". The code could adapt to any filename starting with that prefix. It will use the first one with the prefix founded in the "output" folder.

    It starts with road data (distance to the red light, time stamp red light starts, time stamp red light ends)

    Then follows by front car historical driving data. Each line contains d, v, a, intention.

    The intentino is set to 0 if the front car is decelerating (a\<0)
    
    One trajectory ends with a "end".

    ```
    d2tl=1000,rl_start=30,rl_end=80
    80.00,15.00,0.00,1
    110.00,15.00,0.00,1
    140.00,15.00,0.00,1
    170.00,15.00,0.00,1
    200.00,15.00,0.00,1
    230.00,15.00,0.00,1
    end
    80.00,15.00,0.00,1
    110.00,15.00,0.00,1
    140.00,15.00,0.00,1
    170.00,15.00,0.00,1
    200.00,15.00,0.00,1
    230.00,15.00,0.00,1
    end
    ```

- If you change the number of the prediction steps, or changed the historical data, delete the "w2w_mat.csv"

- Examples of historical data are placed under "test_cases" folder.

- "exam_policy_det_iterative.py" and "exam_policy_sto_iterative.py" are used to test the solver is providing expected policy.

- "show_output.py" could accept different parameters to visualizing the policy. You can use the following commands:
    - "python3 show_output.py value cpu"
    - "python3 show_output.py value gpu"
    - "python3 show_output.py value compare"
    - "python3 show_output.py action cpu"
    - "python3 show_output.py action gpu"
    - "python3 show_output.py action compare"
