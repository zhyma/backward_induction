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
    float dc0 = 80.0;
    int intention = 0;
    // int dk0 = dp_model->get_dist_idx(d0) * dp_model->v.n;
    int dk0 = dp_model->get_dist_idx(d0);
    // int dwk0 = dp_model->get_dist_idx(dc0) * 2 + 0;
    int dck0 = dp_model->get_dist_idx(dc0);

    std::cout << "dwk0 " << dck0 << std::endl;

    std::cout << "prepare work all done" << std::endl;

    if (solver == GPU_SOLVER)
    {
        std::cout << "GPU solver, one step" << std::endl;
        solver_type = "gpu";
        int block_size = 32;

        GPUSolver gpu_solver(dp_model, block_size);

        start = std::clock();
        
        gpu_solver.solve(true, k, d0, v0, dc0, intention);
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
 
        cpu_solver.solve(0, d0, dc0);
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
        // show potential action for different starting velocity
        // for (int i = 0; i < dp_model->v.n; ++i)
        // {
        //     std::cout << cpu_solver.action[i] << std::endl;
        // }

    }

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
