//#include "phy_model.h"
#include "dp_solver.h"
#include "gpu_calc.h"

//initial function
DPSolver::DPSolver(PHYModel * ptr_in, int prob, int sample_rate, int number_of_trials)
{
    ptr_model = ptr_in;
    prob_type = prob;

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

    if (prob_type == ALGEBRAIC)
    {
        create_distribution();
    }
    
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

int DPSolver::write_array_to_csv(int k, float * table)
{
    ofstream out_prob;
    out_prob.open("test_result.csv", ios::out);
    for(int i = 0;i < k; ++i)
    {
        out_prob << table[i] << ',';
    }
            
    out_prob << '\n';

    out_prob.close();
    cout << "Test saved to file." << endl;
    return 0;
}

// "fake" a distribution when \mu is at the center
int DPSolver::create_distribution()
{
    int *histo_bin = new int[w_set.count]{};
    center_distribution = new float[w_set.count]{};
    //mu in the middle
    float mu = (w_set.bound[0] + w_set.bound[1])/2.0;
    float sample = 0;

    // Make sure there is enough sample points
    for (int i = 0; i < w_set.count * 1000; ++i)
    {
        // sample from the model
        sample = ptr_model->mc_disturb(mu);
        // place into the bin
        for (int i = 0; i < w_set.count-1; ++i)
        {
            if (sample <= w_set.list[i] + 0.5/gran)
            {
                histo_bin[i] += 1;
                break;
            }
        }
        if (sample > w_set.list[w_set.count-2] + 0.5/gran)
        {
            histo_bin[w_set.count-1] += 1;
        }
    }
    // get the distribution assume that 
    for (int i =0; i < w_set.count; ++i)
        center_distribution[i] = ((float) histo_bin[i])/((float) w_set.count*1000);

    return 0;
}

// move the center \mu left and right to generate a new distribution
int DPSolver::get_distribution(int wk, float * prob_table)
{
    int center = val_to_idx((w_set.bound[0] + w_set.bound[1])/2.0, &w_set);
    int diff = center - wk;

    // make sure the prob_table is initialized.
    for (int i = 0; i < w_set.count; ++i)
        prob_table[i] = 0.0;

    for (int i = 0; i < w_set.count; ++i)
    {
        if (i-diff <= 0)
        {
            prob_table[0] += center_distribution[i];
        }
        else if (i-diff >= w_set.count-1)
        {
            prob_table[w_set.count-1] += center_distribution[i];
        }
        else
        {
            prob_table[i-diff] = center_distribution[i];
        }
        
    }

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

// Use Monte Carlo method, run the physical model several times, then estimate the probability of states transitions.
int DPSolver::mc_one_stateaction(int k, int xk, int wk, int uk)
{
    float x_ = 0;
    float w_ = 0;
    int  xk_ = 0;
    int  wk_ = 0;
    int  idx = 0;
    float next[2];

    // (s, a, s')
    // initializing all counter as 0
    cnter_table = new int[xw_cnt]();
    // P(s, a, s')
    
    // try iter times, to check the probability
    for (int i = 0;i < iter; ++i)
    {
        ptr_model->linear_model(k, x_set.list[xk], u_set.list[uk], w_set.list[wk], next);
        x_ = next[0];
        w_ = next[1];
        xk_ = val_to_idx(x_, &x_set);
        wk_ = val_to_idx(w_, &w_set);
        idx = xw_idx(xk_, wk_);
        cnter_table[idx] += 1;
    }

    // <x, w> --u--> <x', w'>
    for(int xk_= 0; xk_ < x_set.count; ++xk_)
    {
        for (int wk_=0; wk_ < w_set.count; ++wk_)
        {
            // the number of transit to a certain state divided by the number of all transition
            idx = xw_idx(xk_, wk_);
            float state_cnt = (float) cnter_table[idx];
            float prob = state_cnt/(float) iter;
            prob_table[idx] = prob;
        }
    }
    delete [] cnter_table;

    return 0;
}

float DPSolver::calc_q(int k, int xk, int wk, int uk)
{
    float x_ = 0;
    float w_ = 0;
    int  xk_ = 0;
    int  wk_ = 0;
    int  idx = 0;
    float next[2] = {};
    float sum = 0;


    ptr_model->linear_model(k, x_set.list[xk], w_set.list[wk], u_set.list[uk], next);
    x_ = next[0];
    w_ = next[1];
    xk_ = val_to_idx(x_, &x_set);
    wk_ = val_to_idx(w_, &w_set);

    float *prob_table = new float[w_set.count]{};
    get_distribution(wk, prob_table);
    if (GPU==false){
        for (int i = 0; i < w_set.count; ++i)
        {
            // p*V_{k+1}
            sum += prob_table[i] * value_table[state_idx(k+1, xk_, i)];
        }
    }
    else
    {
        float output = 0;
        intermediate_value(x_set, w_set, prob_table, &(value_table[state_idx(k+1, xk_, 0)]), &output);
    }
    
    
    delete [] prob_table;
    return sum;
}

float DPSolver::estimate_one_step(int k)
{
    clock_t start,end;
    if (k==N)
    {
        // calculate the termianl cost at N=10
        // initial value for V_N is V_N(x)=J_f(x), final cost
        // J_f(x) = (1-x_N)^2
        // final step, no simulation/data is needed
        start = clock();
        if (GPU==false)
        {
            for(int xk = 0; xk < x_set.count; ++xk)
            {
                for (int wk = 0; wk < w_set.count; ++wk)
                {
                    float v = pow(1-x_set.list[xk],2);
                    value_table[state_idx(N, xk, wk)] = v;
                }
            }
        }
        else
        {
            // get states ready
            float *s = new float[x_set.count*w_set.count];
            for (int xk = 0; xk < x_set.count; ++xk)
            {
                for (int wk = 0; wk < w_set.count; ++wk)
                {
                    s[xw_idx(xk, wk)] = x_set.list[xk];
                }
            }
            terminal_value(x_set, w_set, s, &value_table[state_idx(N, 0, 0)]);
            cout << "CUDA at step N" << endl;
        }
        end = clock();
    }
    else if((k >= 0) and (k < N))
    {
        // calculate the running cost
        // searching backward, search for the transition probability one step before, then calculate the min
        // generate probability estimation for intermediate steps

        start = clock();
        // a temporary buffer to save all the result of executing different u for a given xk, wk
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
                    //get the transition probability here
                    if (prob_type == MONTECARLO)
                    {
                        prob_table = new float[xw_cnt]();

                        // from step k to k+1
                        cout << "searching for step " << k << endl;
                        // TODO: NEED TO RE-WORK ON THIS
                        mc_one_stateaction(k, xk, wk, uk);
                        for (int xk_ = 0; xk_ < x_set.count; ++xk_)
                        {
                            for (int wk_ = 0; wk_ < w_set.count; ++wk_)
                            {
                                //<k, x_k> --u_k--> <k+1,x_k+1>
                                int idx = xw_idx(xk_, wk_);
                                float p_z = prob_table[idx];
                                float v_z = value_table[state_idx(k+1, xk_, wk_)];
                                sum += p_z*v_z;
                            }
                        }
                        delete [] prob_table;
                    }
                    else
                    {
                        //cout << "TODO: algebraic way" << endl;
                        sum = calc_q(k, xk, wk, uk);
                    }
                    // z, or x_/x'
                    
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
    for (int i = 0; i < xw_cnt; ++i)
    {
        // xw_idx = xk*w_cnt + wk
        out_value << x_set.list[i/w_set.count] << ";";
        out_value << x_set.list[i%w_set.count] << ",";
    }
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
    for (int i = 0; i < xw_cnt; ++i)
    {
        // xw_idx = xk*w_cnt + wk
        out_action << x_set.list[i/w_set.count] << ";";
        out_action << x_set.list[i%w_set.count] << ",";
    }
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