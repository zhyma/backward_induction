#include <fstream>
#include <string>
#include <sstream>

#include "dp_model.h"
#include "utility.h"

//initial function
DPModel::DPModel(int steps, std::atomic<int> * busy_p_mat)
{
    busy_mat_ptr = busy_p_mat;

    d2tl = 200;
    // the time that the red light will start
    rl_start = 8;
    // the time that the red light will end
    rl_end = rl_start + 8;
    t_tcc = 3;
    m = 1500;

    N = steps;
    dt = 2;
    float bound_d[2] = {.0, 400.0};
    int n_d = 128;
    float bound_v[2] = {.0, 20.0};
    int n_v = 32;
    float bound_a[2] = {-8.0, 2.0};
    int n_a = 32;

    // discretizing x=[d v], w=[d intention], u=[a]
    d.bound[0] = bound_d[0];
    d.bound[1] = bound_d[1];
    d.n = n_d;
    discretize(&d);

    v.bound[0] = bound_v[0];
    v.bound[1] = bound_v[1];
    v.n = n_v;
    discretize(&v);

    a.bound[0] = bound_a[0];
    a.bound[1] = bound_a[1];
    a.n = n_a;
    discretize(&a);

    // x=[d,v]
    x.n = d.n*v.n;
    x.list = new float[x.n]{};
    for (int i = 0; i < d.n; ++i)
        for (int j = 0; j < v.n; ++j)
            x.list[v.n*i + j] = 0;
    
    // u = [a]
    u.n = a.n;
    u.list = new float[a.n]{};
    for (int i = 0; i < u.n; ++i)
        u.list[i] = 0;

    // w = [d, intention]
    w.n = d.n * 2;
    w.list = new float[w.n]{};
    for (int i = 0; i < d.n; ++i)
    {
        w.list[i * 2] = 0;
        w.list[i * 2 + 1] = 0;
    }

    // create <x,w> -u-> x' table here
    state_trans();
    cost_init();

    // prob_table[0] = new float[N*w.n*w.n]{};
    // prob_table[1] = new float[N*w.n*w.n]{};

    check_driving_data();
    
    return;
}

int DPModel::discretize(Set *in)
{
    std::cout << "get variable" << std::endl;
    std::cout << "lower bound " << in->bound[0] << std::endl;
    std::cout << "upper bound " << in->bound[1] << std::endl;
    in->list = new float[in->n];
    for (int i = 0;i < in->n; ++i)
        in->list[i] = in->bound[0] + (in->bound[1]-in->bound[0])/(in->n-1) * i;

    std::cout << "number: " << in->n << std::endl;  
    // std::cout << "list: ";
    // for(int i = 0;i < in->count; ++i)
    //     std::cout << in->list[i] << ", ";
    std::cout << std::endl;

    return 0;
}

int DPModel::state_trans()
{
    s_trans_table = new int[x.n * w.n * u.n]{};
    for (int xk = 0; xk < x.n; ++xk)
    {
        for (int wk = 0; wk < w.n; ++wk)
        {
            for (int uk = 0; uk < u.n; ++uk)
            {
                int xk_ = phy_model(xk, wk, uk);
                // std::cout << "x=" << x << ", w=" << w << ", u=" << u << ", x_=" << x_set.list[xk_] << " (" << xk_ << ")" << std::endl; 
                int idx = xk*(w.n*u.n) + wk*u.n + uk;
                s_trans_table[idx] = xk_;
            }
        }
    }
    return 0;
}

// dx, vx, ax is the given value
int DPModel::phy_model(int xk, int wk, int uk)
{
    float dx = d.list[xk/v.n], vx = d.list[xk%v.n];
    float ax = a.list[uk];
    float d_ = 0, v_ = 0;
    float v_b = v.bound[1];
    if ((vx + ax*dt >= 0) and (vx + ax*dt <= v.bound[1]))
    {
        d_ = dx + vx*dt + 0.5*ax*dt;
        v_ = vx + ax*dt;
    }
    else if (vx + ax*dt > v.bound[1])
    {
        float t1 = (v_b-vx)/ax;
        float t2 = dt - t1;
        d_ = dx + (v_b+vx)*t1/2 + v_b*t2;
        v_ = v_b;
    }
    else 
    {
        d_ = dx - vx*vx/2/ax;
        v_ = 0;
    }
    int dk_ = val_to_idx(d_, &d);
    int vk_ = val_to_idx(v_, &v);
    int xk_ = v.n*dk_ + vk_;
    return xk_;
}

int DPModel::cost_init()
{
    // init cost-to-go mat
    // N_x * N_w * N_u
    running_cost = new float[N*x.n*w.n*u.n]{};
    t_cost = new float[x.n*w.n]{};
    int idx = 0;

    for (int xk = 0; xk < x.n; ++xk)
    {
        for (int wk = 0; wk < w.n; ++wk)
        {
            float dx = d.list[xk/v.n], vx = d.list[xk%v.n];
            float dcx = d.list[wk/2], ix = wk%2;

            for (int uk = 0; uk < u.n; ++uk)
            {
                for (int i = 0; i < N; ++i)
                {
                    idx = i * (x.n*w.n*u.n) + xk*w.n*u.n + wk*u.n + uk;
                    // get the running cost
                    float ax = a.list[uk];

                    int constraint = 0;

                    // constraint 1: safety distance
                    if (dx > dcx - vx*t_tcc)
                    {
                        ++constraint;
                    }

                    // before reaching the red light, and the red light is on
                    if (dx < d2tl && i*dt > rl_start && i*dt < rl_end)
                    {
                        int xk_ = phy_model(xk, wk, uk);
                        float d_ = d.list[xk_/v.n], v_ = v.list[xk%v.n];
                        if (d_ < d2tl)
                        {
                            if (v_*v_ > 2*std::abs(a.bound[0])*(d2tl-d_+0.1))
                                ++constraint;
                        }
                        else
                        {
                            if (v_*v_ > 2*std::abs(a.bound[1])*(d2tl-d_+0.1))
                                ++constraint;
                        }
                    }
                    
                    if (constraint > 0)
                        running_cost[idx] = 1e20;
                    else
                    {
                        // tau_wheel
                        float tan_t = 0;
                        float tau_wheel = m*ax - 0.005*m*g - 0.09*vx*vx - m*g*tan_t;

                        // tau_motor, omega_motor
                        float tau_motor = 0;
                        float omega_motor = 0;
                        float t_ratio = 0;
                        if (vx <= 20)
                        {
                            t_ratio = 1/4.71;
                            omega_motor = 5 * M_PI * vx;
                        }
                        else
                        {
                            t_ratio = 1/2.355;
                            omega_motor = 2.5 * M_PI * vx;
                        }
                        if (tau_wheel >=0)
                            tau_motor = tau_wheel * t_ratio;
                        else
                            tau_motor = tau_wheel * t_ratio * 0.5;
                            
                        float p_motor = omega_motor * tau_motor;
                        float factor = 0;
                        if (tau_motor < 0)
                            factor = 0.97;
                        else
                            factor = 1/0.97;
                        
                        running_cost[idx] =  factor * p_motor * dt;
                    }
                }
            }
            // get the terminal cost
            idx = xk*w.n + wk;
            float term1 = 0.5*m*(v_target*v_target - vx*vx * N)*0.95;
            float term2 = (d_target - dx)*783;
            float term3 = m*g*0*0.95;
            t_cost[idx] = term1 + term2 + term3;
        }
    }

    return 0;
}

int DPModel::check_driving_data()
{
    // check if previously saved raw probability data exist.
    bool p_file_exist = false;
    bool raw_data_exist = false;
    std::vector<std::string> files;
    search_files(&files, "w2w_mat");

    no_of_p = 0;
    // for (int i = 0; i < files.size(); ++i)
    // {
    //     // check file name first
    //     std::cout << files[i].substr(0, 5) << std::endl;
    //     if (files[i].substr(0, 12)!="front_car_data")
    //     {
    //         std::cout << files[i] << " is not a driving data, skip" << std::endl;
    //         continue;
    //     }

    //     std::cout << "checking: " << files[i] << std::endl;
    //     std::string file_name = "./output/" + files[i];
    //     p_mat.push_back(new float[w.n*w.n]{});
    //     // parameter is consistent, load into memory 
    //     std::cout << "Load" << file_name << "from existing data." << std::endl;
    //     p_file_exist = true;
    //     no_of_p++;
    //     // if (FILE *file = fopen(file_name.c_str(), "r"))
    //     // {
    //     //     // if exist, load from the existing one.
    //     //     fclose(file);
    //     //     // load the existing data and generate
    //     //     std::ifstream in_file(file_name, std::ios::in);
    //     //     std::string line_str;
    //     //     bool end_of_file = false;
    //     //     while(end_of_file = false)
    //     //     {
    //     //         // temp for [d, v, a, i] s
    //     //         std::vector<float*> data_temp;
    //     //         bool end_of_trial = false;
    //     //         while(end_of_trial = false)
    //     //         {
    //     //             getline(in_file, line_str);
    //     //             if (line_str == "end")
    //     //                 end_of_trial = true;
    //     //             else
    //     //             {
    //     //                 // get d, v, a, intention
    //     //                 std::stringstream ss_param(line_str); 
    //     //                 std::string arr[4];
    //     //                 for (int k = 0; k < 4; ++k)
    //     //                 {
    //     //                     getline(ss_param, line_str, ',');
    //     //                     arr[k] = line_str;
    //     //                 }
    //     //                 if (std::stof(arr[0]) == w.bound[0] && std::stof(arr[1]) == w.bound[1] && std::stoi(arr[2]) == sample_size)
    //     //                 {
                            
    //     //                     getline(in_file, line_str);
    //     //                     std::stringstream ss_data(line_str); 
    //     //                     for (int j = 0; j < sample_size; ++j)
    //     //                     {
    //     //                         getline(ss_data, line_str, ',');
    //     //                         p_mat_temp[i][j] = std::stof(line_str);
    //     //                     }
    //     //                 }
    //     //             }
    //     //         }
    //     //     }

    //     // }
    // }
    if (p_file_exist == false)
    {
        std::cout << "w to w' file does not exist, create a new one." << std::endl;
        // if not, create a new file and save.
        search_files(&files, "front_car_data");
        int no_of_f = 0;
        for (int f = 0; f < files.size(); ++f)
        {   
            std::string file_name = "./output/" + files[f];

            // read from raw driving data
            if (FILE *file = fopen(file_name.c_str(), "r"))
            {
                // if exist, load from the existing one.
                fclose(file);

                p_mat.push_back(new float[N * w.n * w.n]{});
                int *cnt_mat = new int[N * w.n * w.n]{};

                std::cout << "loading " << file_name << std::endl;
                // load the existing data and generate
                std::ifstream in_file(file_name, std::ios::in);
                std::string line_str;
                bool end_of_file = false;
                while (end_of_file == false)
                {
                    // temp for [d, v, a, i] s
                    int k = 0;
                    int idx;
                    int old_w;
                    bool end_of_trial;
                    std::stringstream ss_param;
                    std::string arr[4];
                    getline(in_file, line_str);
                    if (in_file.eof())
                    {
                        end_of_file = true;
                        continue;
                    }

                    if (line_str.find("end")!=std::string::npos)
                    {
                        end_of_trial = true;
                        k = 0;
                    }
                    else
                    {
                        //prepare for the step 0
                        end_of_trial = false;
                        ss_param.clear();
                        ss_param.str(line_str); 
                        for (int j = 0; j < 4; ++j)
                        {
                            getline(ss_param, line_str, ',');
                            arr[j] = line_str;
                        }
                        int dck = val_to_idx(stof(arr[0]), &d);
                        int i = stoi(arr[3]);
                        old_w = dck*2 + i;
                        std::cout << old_w << std::endl;
                    }
                    while(end_of_trial == false)
                    {
                        getline(in_file, line_str);
                        if (in_file.eof())
                        {
                            end_of_file = true;
                            continue;
                        }
                        ss_param.clear();
                        ss_param.str(line_str); 
                        if (line_str.find("end")!=std::string::npos)
                        {
                            end_of_trial = true;
                            std::cout << std::endl;
                            k = 0;
                        }
                        else
                        {
                            // get d, v, a, intention
                            for (int j = 0; j < 4; ++j)
                            {
                                getline(ss_param, line_str, ',');
                                arr[j] = line_str;
                            }
                            int dck = val_to_idx(stof(arr[0]), &d);
                            int i = stoi(arr[3]);
                            int new_w = dck*2 + i;
                            idx = k*w.n*w.n + w.n*old_w + new_w;
                            cnt_mat[idx] += 1;
                            old_w = new_w;
                            ++k;
                        }
                        std::cout << k << ",";
                    }
                }

                // w->w' samples collecting done, now convert to probabilities
                for (int k = 0; k < N; ++k)
                {
                    for (int wk = 0; wk < w.n; ++wk)
                    {
                        int sum = 0;
                        for (int wk_ = 0; wk_ < w.n; ++wk_)
                        {
                            int idx = k*w.n*w.n + w.n*wk + wk_;
                            sum += cnt_mat[idx];
                        }
                        for (int wk_ = 0; wk_ < w.n; ++wk_)
                        {
                            int idx = k*w.n*w.n + w.n*wk + wk_;
                            if (sum == 0)
                                p_mat[no_of_f][idx] = 0;
                            else
                                p_mat[no_of_f][idx] = float(cnt_mat[idx])/float(sum);
                        }
                    }
                }
                
                // write to files to save w to w' matrices
                std::ofstream out_file;
                out_file.open(("./output/p_mat"+std::to_string(no_of_p)+".csv"), std::ios::out);

                for (int k = 0; k < N; ++k)
                {
                    for (int wk = 0; wk < w.n; ++wk)
                    {
                        for (int wk_ = 0; wk_ < w.n; ++wk_)
                        {
                            int idx = k*w.n*w.n + w.n*wk + wk_;
                            out_file << p_mat[no_of_f][idx] << ",";
                        }
                    }
                }
                out_file.close();

                ++no_of_p;
            }

            
        }
    }

    // create transition probability matrix, with default configuration
    std::cout << "Configured to use probability matrix buffer #0 first." << std::endl;
    // gen_w_trans_mat(0, 0);
}

// By given a value x, find the index of 
int DPModel::val_to_idx(float val, struct Set *ref)
{
    int idx = 0;
    idx = (int) round((val - ref->bound[0])/((ref->bound[1]-ref->bound[0])/(ref->n-1)));
    // make sure it will not be out of boundary because of float accuracy
    idx < ref->bound[0] ? idx = 0 : idx;
    idx > ref->n - 1 ? idx = ref->n -1 : idx;
    return idx;
}

int DPModel::daemon(std::atomic<bool>* running)
{
    int p_type = -1;
    while (*running)
    {
        std::cout << "Change the type of probability to: " << std::endl;
        std::cin >> p_type;
        
        if (p_type >= 0 & p_type < no_of_p)
        {
            // flip the busy_p_mat (ping-pong)
            if (*busy_mat_ptr == 0)
            {
                std::cout << "dp write to buffer channel #1" << std::endl;
                // p_mat 0 is busy, update p_mat 1
                // gen_w_trans_mat(1, p_type);
                *busy_mat_ptr = 1;
            }
            else
            {
                std::cout << "dp write to buffer channel #0" << std::endl;
                // gen_w_trans_mat(0, p_type);
                *busy_mat_ptr = 0;
            }
            std::cout << "The type of probability has been changed to: " << p_type << ". On buffer channel #" << *busy_mat_ptr << std::endl;
        }
        else
        {
            *running = false;
        }
    }

    std::cout << "Exiting DP model daemon" << std::endl;

    return 0;
}