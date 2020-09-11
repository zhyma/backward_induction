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

int DPSolver::solve_one_step(int k)
{
    float *value_table = new float[(ptr_model->N+1)*ptr_model->x_cnt]{};
    float *action_table = new float[ptr_model->N*ptr_model->x_cnt]{};
    // calculate the termianl cost at N=10
    // initial value for V_N is V_N(x)=J_f(x), final cost
    // J_f(x) = (1-x_N)^2
    for(int x = 0; x < ptr_model->x_cnt; ++x)
    {
        float v = pow(1-ptr_model->x_list[x],2);
        value_table[ptr_model->N*ptr_model->x_cnt+x] = v;
    }
    // calculate the running cost
    // searching backward, search for the transition probability one step before, then calculate the min
    // a temporary buffer to save all the result of executing different u for a given xwk
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
    // Save value table and optimal action table to files
    if(true)
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
    }
    
    return 0;
}

// all possible x and u
int DPSolver::solve_whole_model()
{
    float *value_table = new float[(ptr_model->N+1)*ptr_model->x_cnt]{};
    float *action_table = new float[ptr_model->N*ptr_model->x_cnt]{};
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
    // Save value table and optimal action table to files
    if(true)
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
    }
    
    return 0;
}

