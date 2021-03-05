#include <fstream>
#include <string>
#include <sstream>

#include "dp_model.h"
#include "utility.h"

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

int get_param_val(XMLElement* elmt_root, const char* tag)
{
    int param_val;
    const char* param_char = elmt_root->FirstChildElement(tag)->GetText();
    std::stringstream strValue;
    strValue << param_char;
    strValue >> param_val;
    return param_val;
}

//initial function
DPModel::DPModel(int pred_steps, int running_steps)
{
    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(2);
    tinyxml2::XMLDocument doc_xml;
    XMLError err_xml = doc_xml.LoadFile("config.xml");

    if(XML_SUCCESS==err_xml)
    {
        XMLElement* elmt_root = doc_xml.RootElement();

        // Get the granulairty within a unit (discretizing to get states)
        d2tl = get_param_val(elmt_root, "distance_to_trafiic_light");
        std::cout << "distance to the traffic light: " << d2tl << std::endl;
        
        rl_start = get_param_val(elmt_root, "red_light_start");
        std::cout << "red light starts at: " << rl_start << std::endl;

        // the time that the red light will end
        rl_end =get_param_val(elmt_root, "red_light_end");
        std::cout << "red light ends at: " << rl_end << std::endl;
    }
    else
    {
        std::cout << "config.xml read error" << std::endl;

        d2tl = 240;
        // the time that the red light will start
        rl_start = 12;
        // the time that the red light will end
        rl_end = rl_start + 30;
    }

    t_tcc = 3;
    // mass of the vehicle
    m = 1500;
    // Maximum sample points could travel during 10-prediction-step
    n_t = 128;
    // At prediction step 9, the farest position can be reach is 114 (count from 0)
    // There are 14 possible next steps: 0,1,2,...,13
    max_last_step = 13;

    // maximum sample points of the next step (w->w'), for gpu
    n_p = 28;
    // for gpu, at least 32.
    n_p_gpu = 32;

    N_pred = pred_steps;
    N_run = running_steps;
    N_total = N_pred + N_run -1;
    dt = 2;
    
    int n_v = 32;
    // int n_v = 16;
    int n_a = 32;

    v.min = .0;
    v.max = 18.0;
    v.n = n_v;
    discretize(&v);

    a.min = -8.0;
    a.max = 2.0;
    a.n = n_a;
    discretize(&a);

    int n_d = 353;
    d.n = n_d;
    d.list = new float[d.n];
    for (int i = 0; i < d.n; ++i)
        d.list[i] = float(i * v.max * N_pred * dt/(n_t-1));
    d.min = d.list[0];
    d.max = d.list[d.n-1];

    // x=[d,v]
    x.n = d.n*v.n;
    x.list = new float[x.n]{};

    w.n = d.n * 2;
    w.list = new float[w.n]{};

    u.n = a.n;
    u.list = new float[a.n]{};
    for (int i =0; i < u.n; ++i)
        u.list[i] = a.list[i];
    
    // // // create <x,w> -u-> x' table here
    state_trans();
    std::cout << "state transition generated" << std::endl;
    running_cost_init();
    std::cout << "running cost generated" << std::endl;

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
    std::cout << "list: ";
    for(int i = 0;i < in->n; ++i)
        std::cout << in->list[i] << ", ";
    std::cout << std::endl;

    return 0;
}

int DPModel::state_trans()
{
    // std::ofstream out_file;
    // out_file.open("output/transition_test.csv", std::ios::out);
    // out_file << std::setiosflags(std::ios::fixed) << std::setprecision(2);

    // float attr[2] = {.0, .0};

    s_trans_table = new int[x.n * u.n]{};
    for (int xk = 0; xk < x.n; ++xk)
    {
        for (int uk = 0; uk < u.n; ++uk)
        {
            // attr[0] = d.list[xk/v.n];
            // attr[1] = v.list[xk%v.n];
            // out_file << "d=" << d.list[xk/v.n] << ", v=" << v.list[xk%v.n] << ", a=" << u.list[uk] << " -> d=" ;
            int xk_ = phy_model(xk, uk);
            
            int idx = xk*u.n + uk;
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
        int dim[] = {1, x.n, u.n};
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

    d_ > d.max ? (d_ = d.max) : (d_ = d_);
    d_ < d.min ? (d_ = d.min) : (d_ = d_);
    v_ > v.max ? (v_ = v.max) : (v_ = v_);
    v_ < v.min ? (v_ = v.min) : (v_ = v_);

    attr[0] = d_;
    attr[1] = v_;
    return 0;
}

// dx, vx, ax is the given value
int DPModel::phy_model(int xk, int uk)
{
    // dx, vx, ax, d_, v_ are value; xk, wk, uk are index
    float attr[2] = {d.list[xk/v.n], v.list[xk%v.n]};
    float ax = u.list[uk];
    phy_model(attr, ax);

    int dk_ = val_to_idx(attr[0], &d);
    int vk_ = val_to_idx(attr[1], &v);
    int xk_ = dk_*v.n+ vk_;
    return xk_;
}

// By given a value x, find the index of 
int DPModel::val_to_idx(float val, struct Set *ref)
{
    // long long int idx = 0;
    // idx = (long long int) round((val - ref->bound[0])/((ref->bound[1]-ref->bound[0])/(ref->n-1)));
    int idx = 0;
    idx = round((val - ref->min)/((ref->max - ref->min)/(ref->n-1)));
    // make sure it will not be out of boundary because of float accuracy
    // idx < ref->min ? idx = 0 : idx;
    // idx > ref->n - 1 ? idx = ref->n -1 : idx;
    idx < ref->min ? idx = -1 : idx;
    idx > ref->n - 1 ? idx = -1 : idx;
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

int DPModel::running_cost_init()
{
    // N_x * N_w * N_u
    // long long int temp1 = N_total * x.n, temp2 = w.n*u.n;
    int idx = x.n * w.n * u.n;
    std::cout << idx << std::endl;
    r_cost = new float[idx]{};
    r_mask = new unsigned long long int[idx]{};
    idx = 0;

    float cost_min = 1e30, cost_max = 0;

    // std::cout << N_total << ", " << x.n << ", " << w.n << ", " << u.n << std::endl;

    float *running_cost_sample = new float[v.n*a.n];
    for (int xk = 0; xk < x.n; ++xk)
    {
        for (int wk = 0; wk < w.n; ++wk)
        {
            float dx = d.list[xk/v.n], vx = v.list[xk%v.n];
            float dcx = d.list[wk/2], ix = wk%2;

            // std::cout << dx << ", " << vx << "," << dcx << "," << ix << std::endl;

            for (int uk = 0; uk < u.n; ++uk)
            {
                // idx = k * (x.n*w.n*u.n) + xk*w.n*u.n + wk*u.n + uk;
                idx = xk*w.n*u.n + wk*u.n + uk;
                if (idx < 0)
                {
                    std::cout << "In DP model, out of range! idx=" << idx << std::endl;
                    std::cout << "ERROR!" << std::endl;
                }
                // get the running cost
                int ak = uk;
                float ax = a.list[ak];
                // tau_wheel
                // float tan_t = 0;
                float r = 0.3;
                // float tau_wheel = (m*ax + 0.005*m*g + 0.09*vx*vx + m*g*tan_t)*r;

                // // tau_motor, omega_motor
                // float tau_motor = 0;
                // float omega_motor = 0;
                // float t_ratio = 0;
                
                float t1 = 0, t2 = 0;
                float c = 0;
                if (ax >= 0)
                {
                    c = 1.0/0.97*4.71 * (5*M_PI*r);
                    if ((v.max-vx)/ax < dt)
                    {
                        // accelerate then reach the maximum speed
                        t1 = (v.max-vx)/ax;
                        t2 = dt-t1;
                    }
                    else
                    {
                        // accelerate all the time
                        t1 = dt;
                        t2 = 0;
                    }
                    
                }
                else
                {
                    c = 0.97/4.71*0.5 * (5*M_PI*r);
                    if (-vx/ax < dt)
                    {
                        // deccelrate then reach 0
                        t1 = -vx/ax;
                        t2 = dt - t1;
                    }
                    else
                    {
                        // deccelerate all the time
                        t1 = dt;
                        t2 = 0;
                    }
                }
                float g1 = c * (m*ax*vx*t1 + m*ax*ax*t1*t1/2 + 0.005*m*g*vx*t1 + 0.005*m*g*ax*t1*t1/2 + 0.09*POW4(vx+ax*t1)/(4*ax) );
                float g2 = t2 * c * vx*(m*ax + 0.005*m*g + 0.09*vx*vx);
                float cost = g1 + g2;
                
                // // dt is always 2s. t is the actually time (from v->0 or v->v_max)
                // if (ax < 0)
                //     (-vx/ax > dt) ? (t = dt) : (t = -vx/ax);
                // else if (ax > 0)
                //     (((v.max-vx)/ax > dt)) ? (t=dt) : (t = (v.max-vx)/ax);
                // else
                //     t = dt;
                // if (vx <= 20)
                // {
                //     t_ratio = 1/4.71;
                //     omega_motor = 5 * M_PI * vx;
                // }
                // else
                // {
                //     t_ratio = 1/2.355;
                //     omega_motor = 2.5 * M_PI * vx;
                // }
                // if (tau_wheel >=0)
                //     tau_motor = tau_wheel * t_ratio;
                // else
                //     tau_motor = tau_wheel * t_ratio * 0.5;
                    
                // float p_motor = omega_motor * tau_motor;
                // float factor = 0;
                // if (tau_motor < 0)
                //     factor = 0.97;
                // else
                //     factor = 1/0.97;
                // float cost =  factor * p_motor * t;

                r_cost[idx] = cost;

                if (xk/v.n==0 && wk == 0)
                {
                    running_cost_sample[(xk%v.n)*a.n + ak] = cost;
                }

                if (cost > cost_max)
                    cost_max = cost;
                if (cost < cost_min)
                    cost_min = cost;

                // check at each time point whether 
                for (int k = 0; k < N_total; ++k)
                {
                    int constraint = 0;

                    // constraint: speed limit
                    if (vx + dt*ax > v.max)
                    {
                        ++constraint;
                    }

                    // constraint: safety distance
                    if (dx > dcx - vx*t_tcc)
                    {
                        ++constraint;
                    }

                    // constraint: before reaching the red light, and the red light is on
                    if (dx < d2tl && k*dt > rl_start && k*dt < rl_end)
                    {
                        // int xk_ = phy_model(xk, uk);
                        int xk_ = s_trans_table[xk*u.n + uk];
                        float d_ = d.list[xk_/v.n], v_ = v.list[xk_%v.n];
                        if (d_ < d2tl)
                        {
                            if (v_*v_ > 2*std::abs(a.min)*(d2tl-d_+0.1))
                                ++constraint;
                        }
                        else
                        {
                            if (v_*v_ > 2*std::abs(a.max)*(d2tl-d_+0.1))
                                ++constraint;
                        }
                    }
                    
                    if (constraint > 0)
                        r_mask[idx] |= 1<<k;

                }
            }
        }
    }
    if (false)
    {
        std::string filename = "full_r_cost";
        int dim[] = {1, x.n, w.n, u.n};
        mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, r_cost);
        filename = "full_r_mask";
        mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, r_mask);
    }
    if (true)
    {
        std::string filename = "r_cost_va";
        int dim[] = {1, v.n, a.n};
        mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, running_cost_sample);
    }
    std::cout << "the range of running cost: " << cost_min << ", " << cost_max << std::endl;
    return 0;
}

float DPModel::terminal_cost(int dk0, int dk, int vk)
{
    // x stands for value
    // starting position
    float d0x = d.list[dk0];
    // ending position
    float dx  = d.list[dk];
    float vx  = v.list[vk];

    float d_target = d0x + N_pred * dt * v.max;
    float v_target = v.max;

    float term1 = 0;//0.5*m*(v_target*v_target - vx*vx)*0.95;
    float term2 = (d_target - dx)*783;
    float term3 = m*g*0*0.95;
    return term1 + term2 + term3;
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
    if (files.size() > 0)
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
                bool end_of_file = false;
                while (end_of_file == false)
                {
                    // temp for [d, v, a, i] s
                    int k = 0;
                    int idx;
                    int old_wk;
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
                                int new_wk = dck*2 + i;
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

                                idx = (k-1)*w.n*n_p + old_wk*n_p + (new_wk-old_wk);
                                cnt_mat[idx] += 1;
                                old_wk = new_wk;
                            }
                            ++k;
                        }
                        
                    }
                }

                if(false)
                {
                    std::string filename = "count";
                    int dim[] = {N_total, w.n, n_p};
                    mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, cnt_mat);
                }

                std::cout << "dwk range is: " << dwk_min << ", " << dwk_max << std::endl;

                // w->w' samples collecting done, now convert to probabilities
                for (int k = 0; k < N_total; ++k)
                {
                    for (int wk = 0; wk < w.n; ++wk)
                    {
                        int sum = 0;
                        for (int dwk = 0; dwk < n_p; ++dwk)
                        {
                            int idx = k*w.n*n_p + wk*n_p + dwk;
                            sum += cnt_mat[idx];
                        }
                        for (int dwk = 0; dwk < n_p; ++dwk)
                        {
                            int idx = k*w.n*n_p + wk*n_p + dwk;
                            if (sum == 0)
                                prob_table[idx] = 0;
                            else
                            {
                                prob_table[idx] = float(cnt_mat[idx])/float(sum);
                                // if (prob_table[idx] > 0)
                                //     ++valid_p;
                            }
                        }
                    }
                }
                // std::cout << N_total << "," << w.n << "," << n_p << std::endl;
                delete [] cnt_mat;

                std::cout << "generated a new transition probability matrix" << std::endl;
                
                // write to files to save w to w' matrices
                std::ofstream out_file;
                out_file.open(("./output/w2w_mat.csv"), std::ios::out);
                
                int idx = 0;
                for (int k = 0; k < N_total; ++k)
                {
                    for (int wk = 0; wk < w.n; ++wk)
                    {
                        for (int dwk = 0; dwk < n_p; ++dwk)
                        {
                            idx = k*w.n*n_p + wk*n_p + dwk;
                            out_file << prob_table[idx] << ",";
                        }
                    }
                }
                out_file.close();
                std::cout << "idx: " << idx << std::endl;
            }
        }
        
    }
    // std::cout << "non-zero probability: " << valid_p << std::endl;
    
    if(true)
    {
        std::string filename = "prob_full";
        int dim[] = {N_total, w.n, n_p};
        mat_to_file(filename, sizeof(dim)/sizeof(dim[0]), dim, prob_table);
    }
    return 0;
}