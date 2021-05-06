#include "simulate.h"

// return running time
float one_step(int solver, bool save_v, DPModel * dp_model)
{
    std::clock_t start;
    double duration = 0;
    std::string solver_type;
    
    // std::cout << "creating a new DP model is done" << std::endl;
    
    int k = 0;
    // vehicle starting position
    float d0 = 0;
    // vehicle starting velocity
    float v0 = 0;
    // front car starting position
    float dc0 = 0.0;
    // front car starting intention
    int intention = 1;

    // int dk0 = dp_model->get_dist_idx(d0) * dp_model->v.n;
    int dk0 = dp_model->get_dist_idx(d0);
    // int dwk0 = dp_model->get_dist_idx(dc0) * 2 + 0;
    int dck0 = dp_model->get_dist_idx(dc0);

    // std::cout << "dck0 " << dck0 << std::endl;

    if (solver == CPU_SOLVER)
    {
        std::cout << "CPU solver, one step" << std::endl;
        CPUSolver cpu_solver(dp_model, save_v);
        cpu_solver.debug = false;
        solver_type = "cpu";

        start = std::clock();
 
        int a = cpu_solver.solve(0, d0, v0, dc0, intention);
        duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
        std::cout << "CPU time: " << duration << " s" << std::endl;
        
        std::string filename;
        if (save_v)
        {
            filename = std::to_string(dp_model->n_d) + "_";
            filename += std::to_string(dp_model->n_v) + "_";
            filename += std::to_string(dp_model->n_a) + "_";
            filename += "cpu_value";
            long v_dim[] = {1, cpu_solver.n_x, cpu_solver.n_w};
            mat_to_file(filename, sizeof(v_dim)/sizeof(v_dim[0]), v_dim, cpu_solver.value_buffer);
            std::cout << "saved value to: " << filename << std::endl;
        }
        if (true)
        {
            filename = std::to_string(dp_model->n_d) + "_";
            filename += std::to_string(dp_model->n_v) + "_";
            filename += std::to_string(dp_model->n_a) + "_";
            filename += "cpu_action";
            long a_dim[] = {cpu_solver.N, cpu_solver.n_x_s, cpu_solver.n_w_s};
            mat_to_file(filename, sizeof(a_dim)/sizeof(a_dim[0]), a_dim, cpu_solver.action);
            std::cout << "saved action to: " << filename << std::endl;
        }

    }
    else if (solver == GPU_SOLVER)
    {
        std::cout << "GPU solver, one step" << std::endl;
        solver_type = "gpu";
        int block_size = 32;

        GPUSolver gpu_solver(dp_model, block_size);

        start = std::clock();
        
        gpu_solver.solve(k, d0, v0, dc0, intention);
        duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
        std::cout << "GPU time: " << duration << " s" << std::endl;

        if (log)
        {
            std::string filename = "gpu_value";
            long v_dim[] = {gpu_solver.N+1, gpu_solver.n_x, gpu_solver.n_w};
            mat_to_file(filename, sizeof(v_dim)/sizeof(v_dim[0]), v_dim, gpu_solver.value.cpu);

            filename = "gpu_action";
            long a_dim[] = {gpu_solver.N, gpu_solver.n_x_s, gpu_solver.n_w_s};
            mat_to_file(filename, sizeof(a_dim)/sizeof(a_dim[0]), a_dim, gpu_solver.action.cpu);
        }
    }

    return duration;
}

// int run_trials(int trials, int steps, int solver, DPModel * dp_model)
// {
//     float attr[2] = {.0, .0};
//     int ak;
//     float dc0;
//     int intention;
//     int dk0;
//     int dwk0;
//     int block_size = 32;

//     DataWriter save("./output/control.csv");

//     if (solver == CPU_SOLVER)
//     {
//         CPUSolver cpu_solver(dp_model);
//         DataLoader load("./output/front_car_data.csv");

//         int intention = 0;

//         for (int i = 0; i < trials; ++i)
//         {
//             std::cout << "start to solve trial: " << i << std::endl;
//             for(int k = 0; k < steps; ++k)
//             {
//                 // load one from csv
//                 if (load.read_state(dc0, intention) > 0)
//                 {
//                     std::cout << "Front car: " << dc0 << ", " << intention << std::endl;

//                     ak = cpu_solver.solve(k, attr[0], attr[1], dc0, intention);
                    
//                     dp_model->phy_model(attr, dp_model->a.list[ak]);
//                     std::cout << "OUTPUT: d=" << attr[0] << ", v=" << attr[1] << ", a=" << dp_model->a.list[ak] << std::endl;
//                     save.write(dp_model->a.list[ak]);
//                 }
//                 else
//                 {
//                     break;
//                 }
//             }
//             save.write("end");
//             if (load.end_of_file)
//                 break;
//             load.next_trial();
//         }
//     }

//     if (solver == GPU_SOLVER)
//     {
//         GPUSolver gpu_solver(dp_model, block_size);

//         std::string file_name = "./output/front_car_data.csv";
//         // read from raw driving data
//         if (FILE *file = fopen(file_name.c_str(), "r"))
//         {
//             // if exist, load from the existing one.
//             fclose(file);

//             std::cout << "loading " << file_name << std::endl;
//             // load the existing data and generate
//             std::ifstream in_file(file_name, std::ios::in);
//             std::string line_str;
//             std::stringstream ss_param;
//             std::string arr[4];
//             bool end_of_file = false;

//             std::cout << "start to solve" << std::endl;
//             for(int k = 0; k < steps; ++k)
//             {
//                 // load one from csv
//                 std::stringstream ss_param;
//                 std::string arr[4];
//                 getline(in_file, line_str);

//                 if (line_str.find("end")!=std::string::npos)
//                 {
//                     break;
//                 }
//                 else
//                 {
//                     ss_param.clear();
//                     ss_param.str(line_str); 
//                     for (int j = 0; j < 4; ++j)
//                     {
//                         getline(ss_param, line_str, ',');
//                         arr[j] = line_str;
//                     }
//                     std::cout << "FRONT CAR: " << arr[0] << ", " << arr[3] << std::endl;
//                     dc0 = stof(arr[0]);
//                     // int dck = dp_model->get_dist_idx(stof(arr[0]));
//                     intention = stoi(arr[3]);

//                     ak = gpu_solver.solve(k, attr[0], attr[1], dc0, intention);
                    
//                     dp_model->phy_model(attr, dp_model->a.list[ak]);
//                     std::cout << "OUTPUT:" << attr[0] << ", " << attr[1] << ", " << dp_model->a.list[ak] << std::endl;
//                 }
//             }
//         }
//     }
    
//     return 0;
// }
