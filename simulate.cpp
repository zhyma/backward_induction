#include "simulate.h"

// return running time
float one_iter(int solver, bool log, DPModel * dp_model)
{
    std::clock_t start;
    double duration = 0;
    std::string solver_type;
    
    std::cout << "creating a new DP model is done" << std::endl;
    // int N_pred = dp_model->N_pred;
    // int n_x = dp_model->x.n;
    // int n_w = dp_model->w.n;
    // int n_u = dp_model->u.n;

    int k = 0;
    float d0 = 0;
    float v0 = 0;
    float dw0 = 60.0;
    int intention = 0;
    int dk0 = dp_model->get_dist_idx(0) * dp_model->v.n;
    int dwk0 = dp_model->get_dist_idx(60) * 2 + 0;

    std::cout << "prepare work all done" << std::endl;

    if (solver == GPU_SOLVER)
    {
        std::cout << "GPU solver, one step" << std::endl;
        solver_type = "gpu";
        int block_size = 32;

        GPUSolver gpu_solver(dp_model, block_size);

        start = std::clock();
        
        gpu_solver.solve(true, k, d0, v0, dw0, intention);
        duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
        std::cout << std::endl << "GPU time: " << duration << " s" << std::endl;

        if (log)
        {
            std::string filename = "gpu_value";
            int v_dim[] = {gpu_solver.N+1, gpu_solver.n_x, gpu_solver.n_w};
            mat_to_file(filename, sizeof(v_dim)/sizeof(v_dim[0]), v_dim, gpu_solver.value.cpu);

            filename = "gpu_action";
            int a_dim[] = {gpu_solver.N, gpu_solver.n_x, gpu_solver.n_w};
            mat_to_file(filename, sizeof(a_dim)/sizeof(a_dim[0]), a_dim, gpu_solver.action.cpu);
        }
    }

    else if (solver == CPU_SOLVER)
    {
        std::cout << "CPU solver, one step" << std::endl;
        CPUSolver cpu_solver(dp_model);
        solver_type = "cpu";

        start = std::clock();
 
        cpu_solver.solve(0, dk0, dwk0);
        duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
        std::cout << std::endl << "CPU time: " << duration << " s" << std::endl;
        
        if (log)
        {
            std::string filename = "cpu_value";
            int v_dim[] = {cpu_solver.N+1, cpu_solver.n_x, cpu_solver.n_w};
            mat_to_file(filename, sizeof(v_dim)/sizeof(v_dim[0]), v_dim, cpu_solver.value);

            filename = "cpu_action";
            int a_dim[] = {cpu_solver.N, cpu_solver.n_x, cpu_solver.n_w};
            mat_to_file(filename, sizeof(a_dim)/sizeof(a_dim[0]), a_dim, cpu_solver.action);
        }

    }

    // not being used yet
    // else if (solver == COMPARE)
    // {
    //     std::cout << "Run comparison, one step" << std::endl;
    //     int block_size = 32;
    //     // if (block_size >= n_w && block_size > 64)
    //     //     block_size = n_w/2;
    
    //     GPUSolver gpu_solver(dp_model, block_size);

    //     start = std::clock();
    //     gpu_solver.solve(true, 0, dk0, dwk0);
    //     duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    //     std::cout << std::endl << "GPU time: " << duration << " s" << std::endl;

    //     if (log)
    //     {
    //         // std::string filename = "gpu_value";
    //         // int v_dim[3] = {gpu_solver.value.dim[0], gpu_solver.value.dim[1], gpu_solver.value.dim[2]};
    //         // mat_to_file(filename, 3, v_dim, gpu_solver.value.cpu);

    //         // std::string filename = "gpu_action";
    //         // int a_dim[] = {gpu_solver.action.dim[0], gpu_solver.action.dim[1], gpu_solver.action.dim[2]};
    //         // mat_to_file(filename, 3, a_dim, gpu_solver.action.cpu);
    //     }

    //     CPUSolver cpu_solver(dp_model);
    //     start = std::clock();
    //     cpu_solver.solve(0, dk0, dwk0);
    //     duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
    //     std::cout << std::endl << "CPU time: " << duration << " s" << std::endl;

    //     // if (log)
    //     // {
    //     //     std::string filename = "cpu_value";
    //     //     int v_dim[] = {cpu_solver.N+1, cpu_solver.n_x, cpu_solver.n_w};
    //     //     mat_to_file(filename, sizeof(v_dim)/sizeof(v_dim[0]), v_dim, cpu_solver.value);

    //     //     filename = "cpu_action";
    //     //     int a_dim[] = {cpu_solver.N, cpu_solver.n_x, cpu_solver.n_w};
    //     //     mat_to_file(filename, sizeof(a_dim)/sizeof(a_dim[0]), a_dim, cpu_solver.action);
    //     // }

    //     // compare(&cpu_solver, &gpu_solver);
    // }

    return duration;
}

int run_iters(int iters, int solver, DPModel * dp_model)
{
    float attr[2] = {.0, .0};
    int ak;
    float dw0 = 60.0;
    int intention = 1;
    int dk0;
    int dwk0;
    int block_size = 32;
    
    GPUSolver gpu_solver(dp_model, block_size);

    std::string file_name = "./output/front_car_data.csv";
    // read from raw driving data
    if (FILE *file = fopen(file_name.c_str(), "r"))
    {
        // if exist, load from the existing one.
        fclose(file);

        std::cout << "loading " << file_name << std::endl;
        // load the existing data and generate
        std::ifstream in_file(file_name, std::ios::in);
        std::string line_str;
        std::stringstream ss_param;
        std::string arr[4];
        bool end_of_file = false;

        std::cout << "start to solve" << std::endl;
        for(int k = 0; k < iters; ++k)
        {
            // load one from csv
            std::stringstream ss_param;
            std::string arr[4];
            getline(in_file, line_str);

            if (line_str.find("end")!=std::string::npos)
            {
                break;
            }
            else
            {
                ss_param.clear();
                ss_param.str(line_str); 
                for (int j = 0; j < 4; ++j)
                {
                    getline(ss_param, line_str, ',');
                    arr[j] = line_str;
                }
                std::cout << "FRONT CAR: " << arr[0] << ", " << arr[3] << std::endl;
                dw0 = stof(arr[0]);
                int dck = dp_model->get_dist_idx(stof(arr[0]));
                int i = stoi(arr[3]);

                ak = gpu_solver.solve(true, 0, attr[0], attr[1], dw0, intention);
                
                dp_model->phy_model(attr, dp_model->a.list[ak]);
                std::cout << "OUTPUT:" << attr[0] << ", " << attr[1] << ", " << dp_model->a.list[ak] << std::endl;
            }
        }
    }
    
    return 0;
}


// not being used yet
int compare(CPUSolver * c_solver, GPUSolver * g_solver)
{
    int N_pred = c_solver->N;
    int n_x = c_solver->n_x;
    int n_x_s = c_solver->n_x_s;
    int n_w = c_solver->n_w;
    int n_w_s = c_solver->n_w_s;

    //check error
    int error_flag = 0;
    float max_percent = 0;
    int max_per_idx = 0;
    float max_error = 0;
    float *value_error = new float[(N_pred+1)*n_x*n_w]{};
    float *error_percent = new float[(N_pred+1)*n_x*n_w]{};
    int *action_error = new int[N_pred*n_x_s*n_w_s]{};
    int cnt = 0;
    for (int k = 0; k < N_pred+1; ++k)
    {
        for (int xk = 0; xk < n_x; ++xk)
        {
            for (int wk = 0; wk < n_w; ++wk)
            {
                int c_idx = k*n_x*n_w + xk*n_w + wk;
                int g_idx = k*n_x*(g_solver->n_w) + xk *(g_solver->n_w) + wk;
                value_error[c_idx] = fabs(g_solver->value.cpu[g_idx] - c_solver->value[c_idx]);
                if (value_error[c_idx] > max_error)
                    max_error = value_error[c_idx];

                error_percent[c_idx] = value_error[c_idx]/c_solver->value[c_idx];
                if (error_percent[c_idx] > max_percent)
                {
                    max_percent = error_percent[c_idx];
                    max_per_idx = c_idx;
                }
                
                if (error_percent[c_idx] > 1e-6)
                {
                    error_flag ++;
                }
                cnt ++;
            }
        }

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

    return 0;
}