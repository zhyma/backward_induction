//#include "phy_model.h"
#include "dp_solver.h"

//initial function
DPSolver::DPSolver(PHYModel * ptr_in, int sample_rate, int number_of_trials)
{
    ptr_model = ptr_in;
    N = ptr_model->N;
    gran = sample_rate;
    iter = number_of_trials;

    // discretizing x, u, and w
    x_set.bound[0] = ptr_model->x_bound[0];
    x_set.bound[1] = ptr_model->x_bound[1];
    discretize(&x_set);

    w_set.bound[0] = ptr_model->w_bound[0];
    w_set.bound[1] = ptr_model->w_bound[1];
    discretize(&w_set);

    u_set.bound[0] = ptr_model->u_bound[0];
    u_set.bound[1] = ptr_model->u_bound[1];
    discretize(&u_set);

    xw_cnt = x_set.count * w_set.count;
    states_cnt = xw_cnt * N;
    
    cout << "total states: " << states_cnt << endl;

    value_table = new float[(N+1)*xw_cnt]();
    action_table = new float[states_cnt]();
    
    return;
}

int DPSolver::discretize(Set *in)
{
    cout << "get variable" << endl;
    cout << "lower bound " << in->bound[0] << endl;
    cout << "upper bound " << in->bound[1] << endl;
    in->count = (int)(round((in->bound[1]-in->bound[0])*gran+1));
    in->list = new float[in->count];
    for (int i = 0;i < in->count; ++i)
        in->list[i] = in->bound[0] + 1.0/gran * i;

    cout << "number: " << in->count << endl;  
    cout << "list: ";
    for(int i = 0;i < in->count; ++i)
        cout << in->list[i] << ", ";
    cout << endl;

    return 0;
}

int DPSolver::find_min(float *u, int cnt, struct Min_index *min)
{
    int index = 0;
    float value = u[0];
    for(int i = 0;i < cnt; ++i)
    {
        if(u[i] < value)
        {
            value = u[i];
            index = i;
        }
    }
    min->index = index;
    min->value = value;
    return 0;
}

// 2D matrix <x, u>
// find the index of corresponding <x, u> pair
// the full state contains time step k
int DPSolver::xw_idx(int xk, int wk)
{
    int idx = xk * w_set.count + wk;
    return idx;
}

int DPSolver::state_idx(int k, int xk, int wk)
{
    int idx = k * xw_cnt + xw_idx(xk, wk);
    return idx;
    return 0;
}


// For a ONE-STEP-SEARCH
// This is not exactly state-action-state
int DPSolver::sas2idx(int xk, int wk, int uk, int xk_, int wk_)
{
    // state s
    int xw0_idx = xw_idx(xk, wk);
    // state s'
    int xw1_idx = xw_idx(xk_, wk_);
    //int idx = xk*(u_set.count * x_set.count) + uk * x_set.count + xk_;
    int idx = xw0_idx * xw_cnt + uk * xw_cnt + xw1_idx;
    return idx;
}

// By given a value x, find the index of 
int DPSolver::val_to_idx(float val, struct Set *ref)
{
    int idx = 0;
    idx = round((val - ref->bound[0])*gran);
    // make sure it will not be out of boundary because of float accuracy
    idx < ref->bound[0] ? idx = 0 : idx;
    idx > ref->count - 1 ? idx = ref->count -1 : idx;
    return idx;
}

int DPSolver::search_one_step(int k)
{
    float x_ = 0;
    float w_ = 0;
    int xk_ = 0;
    int wk_ = 0;
    int idx = 0;
    float next[2];

    for (int xk = 0;xk < x_set.count; ++xk)
    {
        for (int wk = 0; wk < w_set.count; ++wk)
        {
            // now get a <x,w> pair
            // explore with u
            for (int uk = 0; uk < u_set.count; ++uk)
            {
                // try iter times, to check the probability
                for (int i = 0;i < iter; ++i)
                {
                    ptr_model->linear_model(k, x_set.list[xk], u_set.list[uk], w_set.list[wk], next);
                    x_ = next[0];
                    w_ = next[1];
                    xk_ = val_to_idx(x_, &x_set);
                    xk_ = val_to_idx(w_, &w_set);
                    idx = sas2idx(xk, wk, uk, xk_, wk_);
                    cnter_table[idx] += 1;
                }
            }
        }
    }
    for (int xk = 0;xk < x_set.count; ++xk)
    {
        for (int wk = 0; wk < w_set.count; ++wk)
        {
            // now get a <x,w> pair
            for (int uk = 0; uk < u_set.count; ++uk)
            {
                // <x, w> --u--> <x', w'>
                for(int xk_= 0; xk_ < x_set.count; ++xk_)
                {
                    for (int wk_=0; wk_ < w_set.count; ++wk_)
                    {
                        // the number of transit to a certain state divided by the number of all transition
                        idx = sas2idx(xk, wk, uk, xk_, wk_);
                        float state_cnt = (float) cnter_table[idx];
                        float prob = state_cnt/(float) iter;
                        prob_table[idx] = prob;
                    }
                }
            }
        }
        
    }

    return 0;
}

float DPSolver::solve_one_step(int k)
{
    clock_t start,end;
    if (k==N)
    {
        start = clock();
        // calculate the termianl cost at N=10
        // initial value for V_N is V_N(x)=J_f(x), final cost
        // J_f(x) = (1-x_N)^2
        // final step, no simulation/data is needed
        for(int xk = 0; xk < x_set.count; ++xk)
        {
            for (int wk = 0; wk < w_set.count; ++wk)
            {
                float v = pow(1-x_set.list[xk],2);
                value_table[state_idx(N, xk, wk)] = v;
            }
        }
        end = clock();
    }
    else if((k >= 0) and (k < N))
    {
        // calculate the running cost
        // searching backward, search for the transition probability one step before, then calculate the min
        // generate probability estimation for intermediate steps

        // from step k to k+1
        cout << "searching for step " << k << endl;
        //temp_search = new float[x_set.count * u_set.count]();
        // (s, a, s')
        // initializing all counter as 0
        cnter_table = new int[xw_cnt * u_set.count * xw_cnt]();
        // P(s, a, s')
        prob_table = new float[xw_cnt * u_set.count * xw_cnt]();

        search_one_step(k);

        start = clock();
        // a temporary buffer to save all the result of executing different u for a given xk
        float *u_z_temp = new float[u_set.count]{};

        for (int xk = 0; xk < x_set.count; ++xk)
        {
            for (int wk = 0; wk < w_set.count; ++wk)
            {
                // get a <x, w> pair first
                for (int uk = 0; uk < u_set.count; ++uk)
                {
                    // for each <x, u> (q in RL): l(x,u)+\sum P(z|x,u)V(z)
                    // l(x) = x^2+u^2
                    float x = x_set.list[xk];
                    float u = u_set.list[uk];
                    float l = x*x + u*u;
                    float sum = 0;
                    // z, or x_/x'
                    for (int xk_ = 0; xk_ < x_set.count; ++xk_)
                    {
                        for (int wk_ = 0; wk_ < w_set.count; ++wk_)
                        {
                            //<k, x_k> --u_k--> <k+1,x_k+1>
                            int idx = sas2idx(xk, wk, uk, xk_, wk_);
                            float p_z = prob_table[idx];
                            float v_z = value_table[state_idx(k+1, xk, wk)];
                            sum += p_z*v_z;
                        }
                    }
                    u_z_temp[uk] = l+sum;
                }
                // v = min[l(x,u)+\sum P(z|x,u)V(z)]
                // find the minimium now.
                Min_index min;
                find_min(u_z_temp, u_set.count, &min);
                value_table[state_idx(k, xk, wk)] = min.value;
                action_table[state_idx(k, xk, wk)] = u_set.list[min.index];
            }
        }
        end = clock();

        //delete [] temp_search;
        delete [] cnter_table;
        delete [] prob_table;
    }
    else
    {
        cout << "Error! k="<< k <<" is out of the boundary!" << endl;
    }
    
    // return: time
    return (float) (end-start)/CLOCKS_PER_SEC;
}

// Save value table and optimal action table to files
int DPSolver::write_to_file()
{
    ofstream out_value;
    out_value.open("value.csv", ios::out);
    //title needs to be re-assigned
    // for (int i = 0; i < x_set.count; ++i)
    //     out_value << x_set.list[i] << ",";
    out_value << endl;
    for (int k = 0; k < N+1; k++)
    {
        for (int xk = 0; xk < x_set.count; ++xk)
        {
            for (int wk = 0; wk < w_set.count; ++wk)
                out_value << value_table[state_idx(k, xk, wk)] << ",";
        }
        out_value << endl;
    }
    out_value.close();

    ofstream out_action;
    out_action.open("action.csv", ios::out);
    for (int i = 0; i < x_set.count; ++i)
        out_action << x_set.list[i] << ",";
    out_action << endl;
    for (int k = 0; k < N; k++)
    {
        for (int xk = 0; xk < x_set.count; ++xk)
        {
            for (int wk = 0; wk < w_set.count; ++wk)
                out_action << action_table[state_idx(k, xk, wk)] << ",";
        }
        out_action << endl;
    }
    out_action.close();
    
    return 0;
}