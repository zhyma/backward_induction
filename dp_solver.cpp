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
    x_cnt = (int)(round((ptr_model->x_bound[1]-ptr_model->x_bound[0])*gran+1));
    x_list = new float[x_cnt];
    for(int i = 0;i < x_cnt; ++i)
        x_list[i] = ptr_model->x_bound[0] + 1.0/sample_rate * i;

    u_cnt = (int)(round((ptr_model->u_bound[1]-ptr_model->u_bound[0])*gran+1));
    u_list = new float[u_cnt];
    for(int i = 0;i < u_cnt; ++i)
        u_list[i] = ptr_model->u_bound[0] + 1.0/sample_rate *i;

    value_table = new float[(N+1)*x_cnt]();
    action_table = new float[N*x_cnt]();
    
    return;
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

// For a ONE-STEP-SEARCH
int DPSolver::xu2index(int xk, int uk, int x_)
{
    int idx = xk*(u_cnt * x_cnt) + uk * x_cnt + x_;
    return idx;
}

// For a FULL SEARCH
// By given k, x, u, find the corresponding index (1D dynamic array for convenience)
// For control problem considering the horizon, <k,x> is the state. You may have the same x at different step k.
int DPSolver::kxu2index(int k, int x, int u)
{
    int idx = k * (x_cnt * u_cnt);
    idx += x * u_cnt + u;
    return idx;
}

// For a FULL SEARCH
// By given <s/k*x, a/u>, find next state s_/(k+1)*x_
int DPSolver::x2x_cnt(float x)
{
    int idx = 0;
    idx = round((x - ptr_model->x_bound[0])*gran);
    idx < 0 ? idx = 0 : idx;
    idx > x_cnt - 1 ? idx = x_cnt -1 : idx;
    return idx;
}

int DPSolver::search_one_step(int k)
{
    float x_ = 0;
    int xk_ = 0;
    int idx = 0;

    // search several times
    for (int xk = 0;xk < x_cnt; ++xk)
    {
        for (int uk = 0; uk < u_cnt; ++uk)
        {
            // try iter times, to check the probability
            for (int i = 0;i < iter; ++i)
            {
                x_ = ptr_model->linear_model(k, x_list[xk], u_list[uk], 0.5);
                xk_ = x2x_cnt(x_);
                idx = xu2index(xk, uk, xk_);
                cnter_table[idx] += 1;
            }
        }
    }
    for (int xk = 0;xk < x_cnt; ++xk)
    {
        for (int uk = 0; uk < u_cnt; ++uk)
        {
            for(int xk_= 0; xk_ < x_cnt; ++xk_)
            {
                // the number of transit to a certain state divided by the number of all transition
                idx = xu2index(xk, uk, xk_);
                float state_cnt = (float) cnter_table[idx];
                float prob = state_cnt/(float) iter;
                prob_table[idx] = prob;
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
        for(int x = 0; x < x_cnt; ++x)
        {
            float v = pow(1-x_list[x],2);
            value_table[N * x_cnt+x] = v;
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
        temp_search = new float[x_cnt * u_cnt]();
        // (s, a, s')
        // initializing all counter as 0
        cnter_table = new int[x_cnt * u_cnt * x_cnt]();
        // P(s, a, s')
        prob_table = new float[x_cnt * u_cnt * x_cnt]();

        search_one_step(k);

        start = clock();
        // a temporary buffer to save all the result of executing different u for a given xk
        float *u_z_temp = new float[u_cnt]{};

        for(int xk = 0; xk < x_cnt; ++xk)
        {
            for(int uk = 0; uk < u_cnt; ++uk)
            {
                // for each <x, u> (q in RL): l(x,u)+\sum P(z|x,u)V(z)
                // l(x) = x^2+u^2
                float x = x_list[xk];
                float u = u_list[uk];
                float l = x*x + u*u;
                float sum = 0;
                // z, or x_/x'
                for(int x_ = 0; x_ < x_cnt; ++x_)
                {
                    //<k, x_k> --u_k--> <k+1,x_k+1>
                    int idx = xu2index(xk, uk, x_);
                    float p_z = prob_table[idx];
                    float v_z = value_table[(k+1) * x_cnt + x_];
                    sum += p_z*v_z;
                }
                u_z_temp[uk] = l+sum;
            }
            // v = min[l(x,u)+\sum P(z|x,u)V(z)]
            // find the minimium now.
            Min_index min;
            find_min(u_z_temp, u_cnt, &min);
            value_table[k * x_cnt + xk] = min.value;
            action_table[k * x_cnt + xk] = u_list[min.index];
        }
        end = clock();

        delete [] temp_search;
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

// For a FULL SEARCH
// Search forward one time. Once you search for multiple times, you can get a "grid" model for DP
// Return a matrix. The index is a tuple <x_k, a>, the value corresponding to the tuple is the x_{k+1}
// There are two types of searching:
// 1) from a given x_0, provide one input, get one output, move to the next time step and that the output as the new state
// 2) from all possible x_0, try all input, save all output/next state, then move on to the next step (applying this method here)
// x_k is a combination (tuple) of <k, x>
int DPSolver::global_forward_once(float x0)
{
    float delta = 1.0/gran;
    int idx = 0;
    for (int k = 0; k < N; ++k)
    {
        for (int xk = 0;xk < x_cnt; ++xk)
        {
            for (int uk = 0; uk < u_cnt; ++uk)
            {
                float x = ptr_model->linear_model(k, x_list[xk], u_list[uk], 0.5);
                idx = kxu2index(k, xk, uk);
                temp_search[idx] = x;
            }
        }
    }
    return 0;
}

// For a FULL SEARCH
int DPSolver::estimate_model()
{
    int idx = 0;
    int x_ = 0;

    for(int i = 0;i < iter;++i)
    {
        // search once
        temp_search = new float[(N * x_cnt) * u_cnt]();
        global_forward_once(0);
        // count the state transition for each <x_k, u> pair
        for(int k = 0; k < N; k++)
        {
            for(int xk = 0; xk < x_cnt; xk++)
            {
                for(int uk = 0; uk < u_cnt; uk++)
                {
                    idx = kxu2index(k, xk, uk);
                    x_ = x2x_cnt(temp_search[idx]);
                    cnter_table[idx * (x_cnt+1) + (x_)] += 1;
                    cnter_table[idx * (x_cnt+1) + x_cnt] += 1;
                }
            }
        }
        if((i*100)/iter%5==0)
        {
            cout << (i*100)/iter << "%...";
        }
        delete [] temp_search;
    }
    cout << endl;

    // now get the estimated model
    for(int k = 0;k < N;k++)
    {
        for(int xk =0; xk < x_cnt; ++xk)
        {
            for(int uk=0; uk < u_cnt; ++uk)
            {
                idx = kxu2index(k, xk, uk);
                float all_cnt = (float) cnter_table[idx*(x_cnt+1)+x_cnt];
                for(int x_= 0; x_ < x_cnt; ++x_)
                {
                    // the number of transit to a certain state divided by the number of all transition
                    float state_cnt = (float) cnter_table[idx*(x_cnt+1) + x_];
                    float prob = state_cnt/all_cnt;
                    prob_table[idx*x_cnt + x_] = prob;
                }
            }
        }
    }

    cout << "Model estimation done." << endl;

    return 0;
}

// all possible x and u
int DPSolver::solve_whole_model()
{
    // initializing all counter as 0
    cnter_table = new int[(N * x_cnt) * u_cnt * (x_cnt + 1)]();
    prob_table = new float[(N * x_cnt) * u_cnt * x_cnt]();
    cout << "estimate the whole model first" << endl;
    estimate_model();
    cout << "move on to solver" << endl;
    // calculate the termianl cost at N=10
    // initial value for V_N is V_N(x)=J_f(x), final cost
    // J_f(x) = (1-x_N)^2
    for(int x = 0; x < x_cnt; ++x)
    {
        float v = pow(1-x_list[x],2);
        value_table[N * x_cnt+x] = v;
    }
    // calculate the running cost
    // searching backward
    // a temporary buffer to save all the result of executing different u for a given xk
    float *u_z_temp = new float[u_cnt];
    for(int k = N-1; k >= 0; k--)
    {
        for(int xk = 0; xk < x_cnt; ++xk)
        {
            for(int uk = 0; uk < u_cnt; ++uk)
            {
                // for each <x, u> (q in RL): l(x,u)+\sum P(z|x,u)V(z)
                // l(x) = x^2+u^2
                float x = x_list[xk];
                float u = u_list[uk];
                float l = x*x + u*u;
                float sum = 0;
                // z, or x_/x'
                for(int x_ = 0; x_ < x_cnt; ++x_)
                {
                    //<k, x_k> --u_k--> <k+1,x_k+1>
                    int idx = kxu2index(k, xk, uk);
                    float p_z = prob_table[idx * x_cnt + x_];
                    float v_z = value_table[(k+1) * x_cnt + x_];
                    sum += p_z*v_z;
                }
                u_z_temp[uk] = l+sum;
            }
            // v = min[l(x,u)+\sum P(z|x,u)V(z)]
            // find the minimium now.
            Min_index min;
            find_min(u_z_temp, u_cnt, &min);
            value_table[k * x_cnt + xk] = min.value;
            action_table[k * x_cnt + xk] = u_list[min.index];
        }
    }

    delete [] cnter_table;
    delete [] prob_table;
    
    return 0;
}


// Save value table and optimal action table to files
int DPSolver::write_to_file()
{
    ofstream out_value;
    out_value.open("../value.csv", ios::out);
    for (int i = 0; i < x_cnt; ++i)
        out_value << x_list[i] << ",";
    out_value << endl;
    for (int k = 0; k < N+1; k++)
    {
        for (int xk = 0; xk < x_cnt; ++xk)
        {
            out_value << value_table[k * x_cnt + xk] << ",";
        }
        out_value << endl;
    }
    out_value.close();

    ofstream out_action;
    out_action.open("../action.csv", ios::out);
    for (int i = 0; i < x_cnt; ++i)
        out_action << x_list[i] << ",";
    out_action << endl;
    for (int k = 0; k < N; k++)
    {
        for (int xk = 0; xk < x_cnt; ++xk)
        {
            out_action << action_table[k * x_cnt + xk] << ",";
        }
        out_action << endl;
    }
    out_action.close();
    
    return 0;
}