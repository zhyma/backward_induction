#include <fstream>
#include <string>
#include <sstream>
#include <math.h>

#include "dp_model.h"
#include "utility.h"

#define POW3(x) ((x)*(x)*(x))
#define POW4(x) ((x)*(x)*(x)*(x))

int search_files(std::vector<std::string> *files, std::string search_key)
{
	DIR *dpdf;
    struct dirent *epdf;
    int cnt = 0;

    dpdf = opendir("output");
    if (dpdf != NULL){
        while (epdf = readdir(dpdf))
        {
            std::string name = epdf->d_name;
            std::size_t found = name.find(search_key);
            if (found != std::string::npos)
            {
                files->push_back(name);
                ++cnt;
            }
        }
    }
    closedir(dpdf);
    return cnt;
}

// int get_param_val(XMLElement* elmt_root, const char* tag)
// {
//     int param_val;
//     const char* param_char = elmt_root->FirstChildElement(tag)->GetText();
//     std::stringstream strValue;
//     strValue << param_char;
//     strValue >> param_val;
//     return param_val;
// }

//initial function
DPModel::DPModel(int pred_steps, int running_steps)
{
    test_set = false;

    std::vector<std::string> files;
    search_files(&files, "front_car_data");

    if (files.size() < 1)
    {
        std::cout << "ERROR! Neither has raw driving data, nor transition probability matrix" << std::endl;
        return;
    }

    std::string file_name = "./output/" + files[0];
    DataLoader *load = new DataLoader(file_name);

    d2tl = load->param[0];
    rl_start = load->param[1];
    rl_end = load->param[2];

    delete load;

    N_pred = pred_steps;
    N_run = running_steps;
    N_total = N_pred + N_run -1;
    dt = 2;

    t_ttc = 3;
    // mass of the vehicle
    m = 1500;

    // Maximum sample points could travel during 10-prediction-step
    n_d = 241;
    n_v = 46;
    n_a = 31;

    if (n_d == 31)
    {
        max_last_step = 3;
        n_dc = 36;
        d.n = 84;
    }
    else if (n_d == 61)
    {
        max_last_step = 6;
        n_dc = 71;
        d.n = 168;
    }
    else if (n_d == 121)
    {
        // At prediction step 9, the farest position can be reach is 121-12 (count from 0)
        // There are 13 possible next steps: 0,1,2,...,12
        max_last_step = 12;
        n_dc = 141;
        d.n = 333;
    }
    else if (n_d == 241)
    {
        max_last_step = 24;
        n_dc = 281;
        d.n = 668;
    }
    else if (n_d == 361)
    {
        max_last_step = 36;
        n_dc = 421;
        d.n = 1001;
    }
    else if (n_d == 481)
    {
        max_last_step = 48;
        n_dc = 561;
        d.n = 1334;
    }
    else if (n_d == 601)
    {
        max_last_step = 60;
        n_dc = 701;
        d.n = 1668;
    }
    else if (n_d == 721)
    {
        max_last_step = 72;
        n_dc = 841;
        d.n = 2001;
    }
    // max_last_step = (int)ceil(n_d/N_pred);//13
    // n_dc = n_d + (int)ceil((t_ttc*v.max+3)/(v.max*dt*N_pred)/(n_d-1));//185
    // d.n = n_dc+5;
    // std::cout << "max last step is: " << max_last_step << std::endl;

    // maximum sample points of the next step (w->w')
    // starting from -1, need +1 as offset.
    n_p = (max_last_step+1)*2;

    v.min = .0;
    v.max = 18.0;
    v.n = n_v;
    discretize(&v);

    a.min = -4.0;
    a.max = 2.0;
    a.n = n_a;
    discretize(&a);

    d.list = new float[d.n];
    // std::cout << "distance interval: " << (v.max * N_pred * dt/(n_d-1)) << std::endl;
    for (int i = 0; i < d.n; ++i)
    {
        // d.list[i] = float(i * v.max * N_pred * dt/(n_d-1));
        d.list[i] = float(i * v.max * N_pred * dt/(n_d-1));
        // std::cout << "@" << i << ": " << d.list[i] << ", ";
    }
    // std::cout << std::endl;
    d.min = d.list[0];
    d.max = d.list[d.n-1];

    // x=[d,v]
    // x.n = d.n*v.n;
    x.n = n_d*v.n;
    x.list = new float[x.n]{};

    // w.n = d.n * 2;
    w.n = n_dc * 2;
    w.list = new float[w.n]{};

    u.n = a.n;
    u.list = new float[a.n]{};
    for (int i =0; i < u.n; ++i)
        u.list[i] = a.list[i];
    
    // // // create <x,w> -u-> x' table here
    state_trans();
    // std::cout << "state transition generated" << std::endl;
    running_cost_init();
    // std::cout << "running cost generated" << std::endl;

    prob_table = new float[N_total*w.n*n_p]{};

    check_driving_data();
    // std::cout << "transition probability generated" << std::endl;
    
    return;
}

DPModel::~DPModel(void)
{
    if (d.list)
        delete [] d.list;
    if (v.list)
        delete [] v.list;
    if (a.list)
        delete [] a.list;

    if (x.list)
        delete [] x.list;
    if (u.list)
        delete [] u.list;
    if (w.list)
        delete [] w.list;

    // // save <x,w> -u-> x'
    if (s_trans_table)
        delete [] s_trans_table;
    if (prob_table)
        delete [] prob_table;

    // Save cost-to-go as a matrix
    if (r_cost)
        delete [] r_cost;
    if (r_mask)
        delete [] r_mask;
    // // Save terminal cost as a matrix
}

int DPModel::discretize(Set *in)
{
    // std::cout << "get variable" << std::endl;
    // std::cout << "lower bound " << in->min;
    // std::cout << ", upper bound " << in->max << std::endl;
    in->list = new float[in->n];
    for (int i = 0;i < in->n; ++i)
        in->list[i] = in->min + (in->max - in->min)/(in->n-1) * i;

    // std::cout << "number: " << in->n << std::endl;  
    // std::cout << "list: ";
    // for(int i = 0;i < in->n; ++i)
    //     std::cout << in->list[i] << ", ";
    // std::cout << std::endl;

    return 0;
}

int DPModel::state_trans()
{
    // std::ofstream out_file;
    // out_file.open("output/transition_test.csv", std::ios::out);
    // out_file << std::setiosflags(std::ios::fixed) << std::setprecision(2);

    // float attr[2] = {.0, .0};

    s_trans_table = new long[x.n * u.n]{};
    for (long xk = 0; xk < x.n; ++xk)
    {
        for (int uk = 0; uk < u.n; ++uk)
        {
            // attr[0] = d.list[xk/v.n];
            // attr[1] = v.list[xk%v.n];
            // out_file << "d=" << d.list[xk/v.n] << ", v=" << v.list[xk%v.n] << ", a=" << u.list[uk] << " -> d=" ;
            long xk_ = phy_model(xk, uk);
            
            long idx = xk*u.n + uk;
            s_trans_table[idx] = xk_;

            // out_file << d.list[xk_/v.n] << ", v=" << v.list[xk_%v.n] << std::endl;

            // if (xk_ < 0)
            // {
            //     std::cout << d.list[xk/v.n] << "," << v.list[xk%v.n] << ", " << a.list[uk] << " -> " << xk_/v.n << "," << xk_%v.n  << std::endl;
            // }
        }
    }
    // out_file.close();
    if (debug)
    {
        std::string filename = "tran_full";
        long dim[] = {1, x.n, u.n};
        mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, s_trans_table);
        std::cout << "write transition to file" << std::endl;
    }

    return 0;
}

// dx, vx, ax is the given value
int DPModel::phy_model(float *attr, float ax)
{
    float dx = attr[0];
    float vx = attr[1];
    float d_ = 0, v_ = 0;
    float t = 0;
    if (vx+ax*dt > v.max)
    {
        v_ = v.max;
        t = (v_-vx)/ax;
        d_ = dx + 0.5*(vx+v_)*t + v_*(dt-t);
    }
    else if (vx + ax*dt < 0)
    {
        // or vx+ax*dt < v.min, same
        v_ = 0;
        t = vx/(-ax);
        d_ = dx + 0.5*vx*t;
    }
    else
    {
        v_ = vx + ax*dt;
        d_ = dx + 0.5*(vx+v_)*dt;
    }

    // d_ > d.max ? (d_ = d.max) : (d_ = d_);
    // d_ < d.min ? (d_ = d.min) : (d_ = d_);
    // v_ > v.max ? (v_ = v.max) : (v_ = v_);
    // v_ < v.min ? (v_ = v.min) : (v_ = v_);

    attr[0] = d_;
    attr[1] = v_;
    return 0;
}

// dx, vx is given by xk, ax is the given by uk
int DPModel::phy_model(int xk, int uk)
{
    // dx, vx, ax, d_, v_ are value; xk, wk, uk are index
    float attr[2] = {d.list[xk/v.n], v.list[xk%v.n]};
    float ax = u.list[uk];
    phy_model(attr, ax);

    int dk_ = val_to_idx(attr[0], &d);
    int vk_ = val_to_idx(attr[1], &v);
    long xk_ = dk_*v.n+ vk_;

    // if (xk == 2302 && uk == 20)
    // {
    //     std::cout << "dk " << xk/v.n << std::endl;
    //     std::cout << "vk " << xk%v.n << std::endl;
    //     std::cout << "ak " << uk << std::endl;
    //     std::cout << "d " << d.list[xk/v.n] << std::endl;
    //     std::cout << "v " << v.list[xk%v.n] << std::endl;
    //     std::cout << "a " << u.list[uk] << std::endl;
    //     std::cout << "d_ " << attr[0] << std::endl;
    //     std::cout << "v_ " << attr[1] << std::endl;
    //     std::cout << "2302 to " << xk_ << std::endl;
    // }
    return xk_;
}

// By given a value x, find the index of 
int DPModel::val_to_idx(float val, struct Set *ref)
{
    int idx = 0;
    idx = round((val - ref->min)/((ref->max - ref->min)/(ref->n-1)));
    // make sure it will not be out of boundary because of float accuracy
    // idx < ref->min ? idx = 0 : idx;
    // idx > ref->n - 1 ? idx = ref->n -1 : idx;
    idx < ref->min ? idx = -1 : idx;
    idx > ref->n - 1 ? idx = -1 : idx;

    // if (val <= ref->list[0])
    //     idx = 0;
    // else if (val >= ref->list[ref->n-1])
    //     idx = ref->n-1;
    // else
    // {
    //     for (int i = 0; i < ref->n-1; ++i)
    //     {
    //         if (val > ref->list[i+1])
    //             continue;
    //         else
    //         {
    //             float sub1 = val - ref->list[i];
    //             float sub2 = ref->list[i+1] - val;
    //             if (sub1<=sub2)
    //             {
    //                 idx = i;
    //                 break;
    //             }
    //             else
    //             {
    //                 idx = i+1;
    //                 break;
    //             }
    //         }
    //     }
    // }
    
    return idx;
}

int DPModel::get_dist_idx(float dist)
{
    int idx = 0;
    idx = val_to_idx(dist, &d);
    return idx;
}

int DPModel::get_velc_idx(float velc)
{
    int idx = 0;
    idx = val_to_idx(velc, &v);
    return idx;
}

bool DPModel::front_car_safe(float dx, float vx, float dcx)
{
    // if the front car is safe, return true, else return false
    if (dx > dcx - vx*t_ttc - 3)
        return false;
    else
        return true;
}

bool DPModel::red_light_safe(int k, float dx, float vx, float ax)
{
    // if stop at the redlight is safe, return true, else return false
    bool rl_safe = true;
    if (k*dt > rl_start && k*dt < rl_end)
    {
        // distance constraint
        // a.min is always <0
        if (vx*vx > 2.0*(-a.min)*(d2tl-dx+0.01))
            rl_safe = false;

        // //acceleration constraint (Seems should not exist)
        // if (ax > u_rlmax)
        //     apply_penalty = true;
        // if (k != N_pred)
        // {
        float attr[2] = {dx, vx};
        float ax;
        phy_model(attr, ax);
        float dx_ = attr[0];
        float vx_ = attr[1];
        if (vx_*vx_ > 2.0*(-a.min)*(d2tl-dx_+0.01))
            rl_safe = false;
        // }
    }
    return rl_safe;
}

int DPModel::running_cost_init()
{
    // N_x * N_w * N_u
    // long long int temp1 = N_total * x.n, temp2 = w.n*u.n;
    long idx = 0;
    // std::cout << idx << std::endl;
    r_cost = new float [x.n * w.n * u.n]{};
    r_mask = new long [x.n * w.n * u.n]{};
    long ban_all_time = 0;
    for (int k = 1; k < N_total+1; ++k)
        ban_all_time = ban_all_time | (1<<k);
    // std::cout << "ban_all_time = " << ban_all_time << std::endl;
    bool apply_penalty = false;

    long cost_min = PENALTY, cost_max = 0;

    // std::cout << N_total << ", " << x.n << ", " << w.n << ", " << u.n << std::endl;

    for (long xk = 0; xk < x.n; ++xk)
    {
        for (long wk = 0; wk < w.n; ++wk)
        {
            apply_penalty = false;
            float dx = d.list[xk/v.n], vx = v.list[xk%v.n];
            float dcx = d.list[wk/2], ix = wk%2;

            // constraint: safety distance with the front car
            // true: the front car is safe
            // false: apply penalty
            if (front_car_safe(dx, vx, dcx))
                apply_penalty = false;
            else
                apply_penalty = true;

            // // Seems that this shouldn't exist
            // float u_rlmax = -vx*vx/(2*(d2tl-dx+0.001));
            
            for (int uk = 0; uk < u.n; ++uk)
            {
                idx = xk*w.n*u.n + wk*u.n + uk;
                if (idx < 0)
                {
                    std::cout << "In DP model, out of range! idx=" << idx << std::endl;
                    std::cout << "ERROR!" << std::endl;
                }
                // get the running cost
                int ak = uk;
                float ax = a.list[ak];
                
                float t1 = 0, t2 = 0;
                float c = 0;
                float vx_;
                if (ax > 0)
                {
                    c = (1.0/0.97);
                    if (vx+ax*dt > v.max)
                    {
                        // accelerate then reach the maximum speed
                        t1 = (v.max-vx)/ax;
                        t2 = dt-t1;
                        vx_ = v.max;
                    }
                    else
                    {
                        // accelerate all the time
                        t1 = dt;
                        t2 = 0;
                        vx_ = vx + ax*dt;
                    }
                    
                }
                else if (ax == 0)
                {
                    c = (1.0/0.97);
                    t1 = 0;
                    t2 = dt-t1;
                    vx_ = vx;
                }
                else
                {
                    // ax < 0
                    c = 0.97*0.5;
                    if ((-ax)*dt > vx)
                    {
                        // deccelrate then reach 0
                        t1 = vx/(-ax);
                        t2 = dt - t1;
                        vx_ = v.min;
                    }
                    else
                    {
                        // deccelerate all the time
                        t1 = dt;
                        t2 = 0;
                        vx_ = vx + ax*dt;
                    }
                }
                // integrals
                // velocity is changing
                float g1 = 0;
                if (ax != 0)
                    g1 = c * (m*ax*vx*t1 + 0.5*m*ax*ax*t1*t1 + 0.005*m*g*vx*t1 + 0.5*0.005*m*g*ax*t1*t1 + 0.09*POW4(vx+ax*t1)/(4*ax) );
                    
                // constant velocity (min/max)
                float g2 = c * (m*ax*vx_ + 0.005*m*g*vx_ + 0.09*POW3(vx_)) * t2;

                long cost = long(round(g1 + g2));
                
                if (cost > cost_max)
                    cost_max = cost;
                if (cost < cost_min)
                    cost_min = cost;
                
                r_cost[idx] = cost;

                if (apply_penalty == true)
                    r_mask[idx] = ban_all_time;

                // //constraint: before reaching the red light, and the red light is on
                if (dx < d2tl && apply_penalty == false)
                {
                    for (int k = 1; k < N_total+1; ++k)
                    {
                        float ax = u.list[uk];
                        if (!red_light_safe(k, dx, vx, ax))
                            r_mask[idx] = r_mask[idx] | (1<<k);
                    }
                }
            }
        }
    }
    if (false)
    {
        std::string filename = "full_r_cost";
        long dim[] = {1, x.n, w.n, u.n};
        mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, r_cost);
        filename = "full_r_mask";
        mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, r_mask);
    }
    // std::cout << "the range of running cost: " << cost_min << ", " << cost_max << std::endl;
    return 0;
}

// long DPModel::terminal_cost(int dk0, int dk, int vk)
float DPModel::terminal_cost(long xk, long wk)
{
    int dk = xk/v.n;
    int vk = xk%v.n;
    int dck = wk/2;

    float dx  = d.list[dk];
    float vx  = v.list[vk];
    float dcx = d.list[dck];

    bool t_penalty = false;
    if (!front_car_safe(dx, vx, dcx))
        t_penalty = true;
    if (!red_light_safe(N_pred, dx, vx, v.min))
        t_penalty = true;

    if (t_penalty)
    {
        return PENALTY;
    }
    else
    {
        // float d_target = d0x + N_pred * dt * v.max;
        // float d_target = 10 * dt * v.max;
        float d_target = N_pred * dt * v.max;
        float v_target = v.max;

        float term1 = 0.5*m*(v_target*v_target - vx*vx)*0.95;
        float term2 = (d_target - dx)*783;
        float term3 = m*g*0*0.95;// which is set to 0 for now
        return term1 + term2 + term3;
    }
}

int DPModel::check_driving_data()
{
    // check if previously saved raw probability data exist.
    bool raw_data_exist = false;
    std::vector<std::string> files;
    search_files(&files, "w2w_mat");
    // int valid_p = 0;

    // load pre-processed transition probability matrices
    // Only load the first file.
    if (files.size() > 0 && (new_prob == false))
    {
        std::string file_name = "./output/" + files[0];

        // read from raw driving data
        if (FILE *file = fopen(file_name.c_str(), "r"))
        {
            // if exist, load from the existing one.
            fclose(file);
            std::cout << "loading transition probability matrix " << file_name << std::endl;

            std::ifstream in_file(file_name, std::ios::in);         
            if (in_file.eof())
                std::cout << "empty file" << std::endl;
            else
            {
                std::string line_str;
                // ignore the first line (parameters)
                getline(in_file, line_str);
                getline(in_file, line_str);
                std::stringstream ss_param;
                ss_param.clear();
                ss_param.str(line_str);
                for (int idx = 0; idx < N_total * w.n * n_p; ++idx)
                {
                    getline(ss_param, line_str, ',');
                    prob_table[idx] = std::stof(line_str);
                    // if (prob_table[idx] > 0)
                    //     ++valid_p;
                }
                // std::cout << "probability matrix loaded and prepared" << std::endl;
            }
        }
    }
    // load raw driving data, generate transition probability matrices
    else
    {
        std::cout << "w to w' file does not exist, create a new one." << std::endl;
        // if not, create a new file and save.
        search_files(&files, "front_car_data");

        if (files.size() < 1)
        {
            std::cout << "ERROR! Neither has raw driving data, nor transition probability matrix" << std::endl;
        }
        // only load the first file
        else
        {   
            std::string file_name = "./output/" + files[0];

            int dwk_min=n_p, dwk_max=0;
            // read from raw driving data
            if (FILE *file = fopen(file_name.c_str(), "r"))
            {
                // if exist, load from the existing one.
                fclose(file);

                int *cnt_mat = new int[N_total * w.n * n_p]{};

                std::cout << "loading " << file_name << std::endl;
                // load the existing data and generate
                std::ifstream in_file(file_name, std::ios::in);
                std::string line_str;

                // skip the 1st line, which is the parameters
                getline(in_file, line_str);

                bool end_of_file = false;
                while (end_of_file == false)
                {
                    // temp for [d, v, a, i] s
                    int k = 0;
                    long idx;
                    long old_wk;
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
                        // if arr[0] is out of range, dck is -1, old_wk is -2 or -1 (always <0)
                        old_wk = dck*2 + i;
                        // std::cout << old_wk << std::endl;
                        // std::cout << "load" << k << std::endl;
                        ++k;
                    }
                    while(end_of_trial == false)
                    {
                        //work on the rest steps
                        getline(in_file, line_str);
                        if (in_file.eof())
                        {
                            end_of_trial = true;
                            end_of_file = true;
                            continue;
                        }
                        ss_param.clear();
                        ss_param.str(line_str); 
                        if (line_str.find("end")!=std::string::npos)
                        {
                            end_of_trial = true;
                            // std::cout << "find end" << std::endl;
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
                            if (k <= N_total)
                            {
                                // std::cout << "load " << k << ",";
                                int dck = val_to_idx(stof(arr[0]), &d);
                                int i = stoi(arr[3]);
                                long new_wk = dck*2 + i;
                                // std::cout << new_wk << std::endl;

                                // out of bound
                                if ((old_wk < 0) || (new_wk < 0))
                                {
                                    old_wk = new_wk;
                                    continue;
                                }

                                if (new_wk - old_wk > dwk_max)
                                    dwk_max = new_wk - old_wk;
                                if (new_wk - old_wk < dwk_min)
                                    dwk_min = new_wk - old_wk;

                                // if (new_wk - old_wk < 0)
                                // {
                                //     std::cout << "-1! k: " << k;
                                //     std::cout << ", old wk: " << old_wk;
                                //     std::cout << ", new wk: " << new_wk;
                                //     std::cout << ", value: " << arr[0];
                                //     std::cout << ", value: " << arr[1];
                                //     std::cout << ", value: " << arr[2];
                                //     std::cout << ", value: " << arr[3] << std::endl;
                                // }
                                // if (new_wk - old_wk > 23)
                                // {
                                //     std::cout << "24! k: " << k;
                                //     std::cout << ", old wk: " << old_wk;
                                //     std::cout << ", new wk: " << new_wk;
                                //     std::cout << ", value: " << arr[0];
                                //     std::cout << ", value: " << arr[1];
                                //     std::cout << ", value: " << arr[2];
                                //     std::cout << ", value: " << arr[3] << std::endl;
                                // }

                                // idx = (k-1)*w.n*n_p + old_wk*n_p + (new_wk-old_wk);
                                // dwk ranges from -1 to 25
                                // -1: same position, intention 1->0
                                // 25: maximum speed, intention 0->1
                                // use 1 as offset to compensate the -1 case
                                idx = (k-1)*w.n*n_p + old_wk*n_p + (new_wk-old_wk+1);
                                cnt_mat[idx] += 1;
                                old_wk = new_wk;
                            }
                            ++k;
                        }
                        
                    }
                }

                // std::cout << "min dwk is: " << dwk_min << std::endl;
                // std::cout << "max dwk is: " << dwk_max << std::endl;

                if(false)
                {
                    std::string filename = "count";
                    long dim[] = {N_total, w.n, n_p};
                    mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, cnt_mat);
                }

                // std::cout << "dwk range is: " << dwk_min << ", " << dwk_max << std::endl;

                // w->w' samples collecting done, now convert to probabilities
                for (int k = 0; k < N_total; ++k)
                {
                    for (long wk = 0; wk < w.n; ++wk)
                    {
                        int sum = 0;
                        for (int dwk = 0; dwk < n_p; ++dwk)
                        {
                            long idx = k*w.n*n_p + wk*n_p + dwk;
                            sum += cnt_mat[idx];
                        }
                        for (int dwk = 0; dwk < n_p; ++dwk)
                        {
                            long idx = k*w.n*n_p + wk*n_p + dwk;
                            if (sum == 0)
                                prob_table[idx] = 0;
                            else
                                prob_table[idx] = float(cnt_mat[idx])/float(sum);
                        }
                    }
                }
                // std::cout << N_total << "," << w.n << "," << n_p << std::endl;
                delete [] cnt_mat;

                std::cout << "generated a new transition probability matrix" << std::endl;
                
                // // write to files to save w to w' matrices
                // std::ofstream out_file;
                // out_file.open(("./output/w2w_mat.csv"), std::ios::out);
                
                // int idx = 0;
                // for (int k = 0; k < N_total; ++k)
                // {
                //     for (int wk = 0; wk < w.n; ++wk)
                //     {
                //         for (int dwk = 0; dwk < n_p; ++dwk)
                //         {
                //             idx = k*w.n*n_p + wk*n_p + dwk;
                //             out_file << prob_table[idx] << ",";
                //         }
                //     }
                // }
                // out_file.close();
                // // std::cout << "idx: " << idx << std::endl;
            }
        }
        
    }
    // std::cout << "non-zero probability: " << valid_p << std::endl;
    
    if(debug)
    {
        std::string filename = "prob_full";
        long dim[] = {N_total, w.n, n_p};
        mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, prob_table);
    }
    return 0;
}

int DPModel::action_filter(float v0, int a_idx)
{
    float a_out = a.list[a_idx];
    int ak = a_idx;
    if (v0+a_out*dt < v.min)
    {
        //from smallest to a larger one
        for (; ak < a.n-1; ++ak)
        {
            if (v0 + a.list[ak+1]*dt > v.min)
                break;
        }
    }
    else if (v0 + a_out*dt > v.max)
    {
        for (; ak > 0; --ak)
        {
            if (v0 + a.list[ak-1]*dt < v.max)
                break;
        }
    }
    return ak;
}