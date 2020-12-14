#include <fstream>
#include <string>
#include <sstream> 

#include "dp_model.h"

//initial function
DPModel::DPModel(PHYModel * ptr_in, int steps, int n_x, int n_w, int n_u, std::atomic<int> * busy_p_mat)
{
    ptr_model = ptr_in;
    //sample size
    sample_size = 1024;
    p_mat_temp = new float[sample_size]{};

    busy_mat_ptr = busy_p_mat;

    N = steps;
    // discretizing x, u, and w
    x_set.bound[0] = ptr_model->x_bound[0];
    x_set.bound[1] = ptr_model->x_bound[1];
    x_set.count = n_x;
    discretize(&x_set);

    w_set.bound[0] = ptr_model->w_bound[0];
    w_set.bound[1] = ptr_model->w_bound[1];
    w_set.count = n_w;
    discretize(&w_set);

    u_set.bound[0] = ptr_model->u_bound[0];
    u_set.bound[1] = ptr_model->u_bound[1];
    u_set.count = n_u;
    discretize(&u_set);

    xw_cnt = x_set.count * w_set.count;
    states_cnt = xw_cnt * N;
    
    std::cout << "total states: " << states_cnt << std::endl;

    // create <x,w> -u-> x' table here
    state_trans();

    prob_table[0] = new float[N*w_set.count*w_set.count]{};
    prob_table[1] = new float[N*w_set.count*w_set.count]{};

    // check if previously saved raw probability data exist.
    std::string file_name = "raw_prob.csv";
    bool file_exist = false;
    if (FILE *file = fopen(file_name.c_str(), "r"))
    {
        // if exist, load from the existing one.
        fclose(file);
        // load the existing probability
        std::ifstream in_file(file_name, std::ios::in);
        std::string line_str;
        // get lower, upper, gran. check parameters
        getline(in_file, line_str);
        std::stringstream ss_param(line_str); 
        std::string arr[3];
        for (int i = 0; i < 3; ++i)
        {
            getline(ss_param, line_str, ',');
            arr[i] = line_str;
        }
        if (std::stof(arr[0]) == w_set.bound[0] && std::stof(arr[1]) == w_set.bound[1] && std::stoi(arr[2]) == sample_size)
        {
            // parameter is consistent, load into memory 
            std::cout << "Load raw distribution from existing data." << std::endl;
            file_exist = true;
            getline(in_file, line_str);
            std::stringstream ss_data(line_str); 
            for (int i = 0; i < sample_size; ++i)
            {
                getline(ss_data, line_str, ',');
                p_mat_temp[i] = std::stof(line_str);
            }
        } 
        else
            file_exist = false;
    } 
    if (file_exist == false)
    {
        // if not, create a new file and save.
        std::cout << "Raw distribution file does not exist, create a new one." << std::endl;
        distribution();
        std::ofstream out_file;
        out_file.open(file_name, std::ios::out);

        out_file << w_set.bound[0] << ",";
        out_file << w_set.bound[1] << ",";
        out_file << sample_size << std::endl;
        for (int i = 0; i < sample_size; i++)
            out_file << p_mat_temp[i] << ",";
            
        out_file << std::endl;
        out_file.close();
    }   
    
    // create transition probability matrix, with default configuration
    std::cout << "Configured to use probability matrix buffer #0 first." << std::endl;
    gen_w_trans_mat(0, 0);
    
    return;
}

int DPModel::discretize(Set *in)
{
    std::cout << "get variable" << std::endl;
    std::cout << "lower bound " << in->bound[0] << std::endl;
    std::cout << "upper bound " << in->bound[1] << std::endl;
    in->list = new float[in->count];
    for (int i = 0;i < in->count; ++i)
        in->list[i] = in->bound[0] + (in->bound[1]-in->bound[0])/(in->count-1) * i;

    std::cout << "number: " << in->count << std::endl;  
    // std::cout << "list: ";
    // for(int i = 0;i < in->count; ++i)
    //     std::cout << in->list[i] << ", ";
    std::cout << std::endl;

    return 0;
}

// 2D matrix <x, u>
// find the index of corresponding <x, u> pair
// the full state contains time step k
int DPModel::xw_idx(int xk, int wk)
{
    int idx = xk * w_set.count + wk;
    return idx;
}

int DPModel::state_idx(int k, int xk, int wk)
{
    int idx = k * xw_cnt + xw_idx(xk, wk);
    return idx;
    return 0;
}

// By given a value x, find the index of 
int DPModel::val_to_idx(float val, struct Set *ref)
{
    int idx = 0;
    idx = (int) round((val - ref->bound[0])/((ref->bound[1]-ref->bound[0])/(ref->count-1)));
    // make sure it will not be out of boundary because of float accuracy
    idx < ref->bound[0] ? idx = 0 : idx;
    idx > ref->count - 1 ? idx = ref->count -1 : idx;
    return idx;
}

int DPModel::state_trans()
{
    s_trans_table = new int[x_set.count * w_set.count * u_set.count]{};
    for (int xk = 0; xk < x_set.count; ++xk)
    {
        for (int wk = 0; wk < w_set.count; ++wk)
        {
            for (int uk = 0; uk < u_set.count; ++uk)
            {
                float x = x_set.list[xk];
                float w = w_set.list[wk];
                float u = u_set.list[uk];

                float x_ = ptr_model->linear_model(x,w,u);
                int xk_ = val_to_idx(x_, &x_set);
                // std::cout << "x=" << x << ", w=" << w << ", u=" << u << ", x_=" << x_set.list[xk_] << " (" << xk_ << ")" << std::endl; 
                int idx = xk*(w_set.count*u_set.count) + wk*u_set.count + uk;
                s_trans_table[idx] = xk_;
            }
        }
    }
    return 0;
}

int DPModel::distribution()
{
    float w_center = (w_set.bound[0] + w_set.bound[1])/2.0;
    float gran = (w_set.bound[1] - w_set.bound[0])/(float) sample_size;
    int *list = new int[sample_size]{};
    int i = 0;
    while (i < sample_trials)
    {
        // get random number with normal distribution using gen as random source
        float w_ = ptr_model->next_w(w_center);

        if (w_ > w_set.bound[0] + gran/2.0 && w_ < w_set.bound[1] - gran/2.0)
        {
            int no = 1 + (int) ((w_ - w_set.bound[0]-gran/2.0)/gran);
            ++list[no];
            ++i;
        }
    }
    for (int i = 0;i < sample_size; ++i)
        p_mat_temp[i] = (float) list[i]/(float) sample_trials;

    return 0;
}

int DPModel::gen_w_trans_mat(int update_mat, int prob_type)
{
    if (update_mat != 0 && update_mat != 1)
        update_mat = 0;

    int n_w = w_set.count;
    float *prob_temp = new float[n_w]{};
    int cnt = sample_size/n_w;

    for (int i = 0; i < sample_size; ++i)
        prob_temp[i/cnt] += p_mat_temp[i];
        
    std::cout << std::endl;

    for (int k = 0; k < N; ++k)
    {
        int offset = k*n_w*n_w;
        for (int i = 0; i < n_w; ++i)
        {
            float sum = 0;
            // given w_k
            if (i < n_w/2)
            {
                for (int j = 0, p_idx = n_w/2-i-1; j < n_w/2+i+1; ++j, ++p_idx)
                {
                    prob_table[update_mat][offset + i*n_w + j] = prob_temp[p_idx];
                    sum += prob_temp[p_idx];
                    // std::cout << j << "+" << p_idx << ",";
                }
                // std::cout << std::endl;
            }
            else
            {
                for (int j = i-n_w/2, p_idx = 0; j < n_w; ++j, ++p_idx)
                {
                    prob_table[update_mat][offset + i*n_w + j] = prob_temp[p_idx];
                    sum += prob_temp[p_idx];
                    // std::cout << j << "+" << p_idx << ",";
                }
                // std::cout << std::endl;
            }
            
            for (int j = 0; j < n_w; ++j)
                prob_table[update_mat][offset + i*n_w + j] /= sum;

        }
    }
    return 0;
}

int DPModel::daemon(std::atomic<bool>* running)
{
    int p_type = -1;
    while (*running)
    {
        std::cout << "Change the type of probability to: " << std::endl;
        std::cin >> p_type;
        
        if (p_type == 0)
        {
            // flip the busy_p_mat (ping-pong)
            if (*busy_mat_ptr == 0)
            {
                std::cout << "dp write to buffer channel #1" << std::endl;
                // p_mat 0 is busy, update p_mat 1
                gen_w_trans_mat(1, p_type);
                *busy_mat_ptr = 1;
            }
            else
            {
                std::cout << "dp write to buffer channel #0" << std::endl;
                gen_w_trans_mat(0, p_type);
                *busy_mat_ptr = 0;
            }
            std::cout << "The type of probability has been changed to: " << p_type << ". On buffer channel #" << *busy_mat_ptr << std::endl;
        }
        
        if (p_type > 5)
        {
            *running = false;
        }
        
    }

    std::cout << "Exiting DP model daemon" << std::endl;

    return 0;
}