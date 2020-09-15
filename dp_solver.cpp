#include "dp_solver.h"

DPSolver::DPSolver(DPModel * ptr_in)
{
    ptr_model = ptr_in;
    value_table = new float[(ptr_model->N+1)*ptr_model->x_cnt]{};
    action_table = new float[ptr_model->N*ptr_model->x_cnt]{};
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

int DPSolver::xu2index(int xk, int uk, int x_)
{
    int idx = xk*(ptr_model->u_cnt * ptr_model->x_cnt) + uk * ptr_model->x_cnt + x_;
    return idx;
}

int DPSolver::search_one_step(int k, int iter)
{
    temp_search = new float[ptr_model->x_cnt * ptr_model->u_cnt];
    // (s, a, s')
    // initializing all counter as 0
    cnter_table = new int[ptr_model->x_cnt * ptr_model->u_cnt * ptr_model->x_cnt] ();
    // P(s, a, s')
    prob_table = new float[ptr_model->x_cnt * ptr_model->u_cnt * ptr_model->x_cnt];
    float x_ = 0;
    int xk_ = 0;
    int idx = 0;

    // search several times
    for (int xk = 0;xk < ptr_model->x_cnt; ++xk)
    {
        for (int uk = 0; uk < ptr_model->u_cnt; ++uk)
        {
            // try iter times, to check the probability
            for (int i = 0;i < iter; ++i)
            {
                x_ = ptr_model->ptr_model->linear_model(k, ptr_model->x_list[xk], ptr_model->u_list[uk], 0.5);
                xk_ = ptr_model->x2x_cnt(x_);
                idx = xu2index(xk, uk, xk_);
                cnter_table[idx] += 1;
            }
        }
    }
    for (int xk = 0;xk < ptr_model->x_cnt; ++xk)
    {
        for (int uk = 0; uk < ptr_model->u_cnt; ++uk)
        {
            for(int xk_= 0; xk_ < ptr_model->x_cnt; ++xk_)
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
    if (k==ptr_model->N)
    {
        start = clock();
        // calculate the termianl cost at N=10
        // initial value for V_N is V_N(x)=J_f(x), final cost
        // J_f(x) = (1-x_N)^2
        // final step, no simulation/data is needed
        for(int x = 0; x < ptr_model->x_cnt; ++x)
        {
            float v = pow(1-ptr_model->x_list[x],2);
            value_table[ptr_model->N*ptr_model->x_cnt+x] = v;
        }
        end = clock();
    }
    else if((k >= 0) and (k < ptr_model -> N))
    {
        // calculate the running cost
        // searching backward, search for the transition probability one step before, then calculate the min
        // generate probability estimation for intermediate steps

        // from step k to k+1
        cout << "searching for step " << k << endl;
        search_one_step(k, 1000);

        start = clock();
        // a temporary buffer to save all the result of executing different u for a given xk
        float *u_z_temp = new float[ptr_model->u_cnt]{};

        for(int xk = 0; xk < ptr_model->x_cnt; ++xk)
        {
            for(int uk = 0; uk < ptr_model->u_cnt; ++uk)
            {
                // for each <x, u> (q in RL): l(x,u)+\sum P(z|x,u)V(z)
                // l(x) = x^2+u^2
                float x = ptr_model->x_list[xk];
                float u = ptr_model->u_list[uk];
                float l = x*x + u*u;
                float sum = 0;
                // z, or x_/x'
                for(int x_ = 0; x_ < ptr_model->x_cnt; ++x_)
                {
                    //<k, x_k> --u_k--> <k+1,x_k+1>
                    int idx = xu2index(xk, uk, x_);
                    float p_z = prob_table[idx];
                    float v_z = value_table[(k+1)*ptr_model->x_cnt + x_];
                    sum += p_z*v_z;
                }
                u_z_temp[uk] = l+sum;
            }
            // v = min[l(x,u)+\sum P(z|x,u)V(z)]
            // find the minimium now.
            Min_index min;
            find_min(u_z_temp, ptr_model->u_cnt, &min);
            value_table[k*ptr_model->x_cnt + xk] = min.value;
            action_table[k*ptr_model->x_cnt + xk] = ptr_model->u_list[min.index];
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
    out_value.open("../value.csv", ios::out);
    for (int i = 0; i < ptr_model->x_cnt; ++i)
        out_value << ptr_model->x_list[i] << ",";
    out_value << endl;
    for (int k = 0; k < ptr_model->N+1; k++)
    {
        for (int xk = 0; xk < ptr_model->x_cnt; ++xk)
        {
            out_value << value_table[k*ptr_model->x_cnt + xk] << ",";
        }
        out_value << endl;
    }
    out_value.close();

    ofstream out_action;
    out_action.open("../action.csv", ios::out);
    for (int i = 0; i < ptr_model->x_cnt; ++i)
        out_action << ptr_model->x_list[i] << ",";
    out_action << endl;
    for (int k = 0; k < ptr_model->N; k++)
    {
        for (int xk = 0; xk < ptr_model->x_cnt; ++xk)
        {
            out_action << action_table[k*ptr_model->x_cnt + xk] << ",";
        }
        out_action << endl;
    }
    out_action.close();
    
    return 0;
}

// all possible x and u
int DPSolver::solve_whole_model()
{
    // calculate the termianl cost at N=10
    // initial value for V_N is V_N(x)=J_f(x), final cost
    // J_f(x) = (1-x_N)^2
    for(int x = 0; x < ptr_model->x_cnt; ++x)
    {
        float v = pow(1-ptr_model->x_list[x],2);
        value_table[ptr_model->N*ptr_model->x_cnt+x] = v;
    }
    // calculate the running cost
    // searching backward
    // a temporary buffer to save all the result of executing different u for a given xk
    float *u_z_temp = new float[ptr_model->u_cnt];
    for(int k = ptr_model->N-1; k >= 0; k--)
    {
        for(int xk = 0; xk < ptr_model->x_cnt; ++xk)
        {
            for(int uk = 0; uk < ptr_model->u_cnt; ++uk)
            {
                // for each <x, u> (q in RL): l(x,u)+\sum P(z|x,u)V(z)
                // l(x) = x^2+u^2
                float x = ptr_model->x_list[xk];
                float u = ptr_model->u_list[uk];
                float l = x*x + u*u;
                float sum = 0;
                // z, or x_/x'
                for(int x_ = 0; x_ < ptr_model->x_cnt; ++x_)
                {
                    //<k, x_k> --u_k--> <k+1,x_k+1>
                    int idx = ptr_model->kxu2index(k, xk, uk);
                    float p_z = ptr_model->prob_table[idx*ptr_model->x_cnt + x_];
                    float v_z = value_table[(k+1)*ptr_model->x_cnt + x_];
                    sum += p_z*v_z;
                }
                u_z_temp[uk] = l+sum;
            }
            // v = min[l(x,u)+\sum P(z|x,u)V(z)]
            // find the minimium now.
            Min_index min;
            find_min(u_z_temp, ptr_model->u_cnt, &min);
            value_table[k*ptr_model->x_cnt + xk] = min.value;
            action_table[k*ptr_model->x_cnt + xk] = ptr_model->u_list[min.index];
        }
    }
    
    return 0;
}

