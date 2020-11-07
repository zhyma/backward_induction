#include <string>
#include <iomanip>
#include "dp_model.h"

int write_to_file(DPModel * model, std::string solver_type, float *v, int * a)
{
    int N = model->N;
    int n_x = model->x_set.count;
    int n_w = model->w_set.count;
    int n_u = model->u_set.count;

    std::ofstream out_value;
    out_value.open("output/" + solver_type + "_value.csv", std::ios::out);
    out_value << std::setiosflags(std::ios::fixed) << std::setprecision(2);

    //title needs to be re-assigned
    for (int i = 0; i < model->xw_cnt; ++i)
    {
        // xw_idx = xk*w_cnt + wk
        out_value << model->x_set.list[i/n_w] << ";";
        out_value << model->x_set.list[i%n_w] << ",";
    }
    out_value << std::endl;

    for (int k = 0; k <N+1; k++)
    {
        for (int xk = 0; xk < n_x; ++xk)
        {
            for (int wk = 0; wk < n_w; ++wk)
            {
                int idx = k*(n_x*n_w) + xk*n_w + wk;
                out_value << v[idx] << ",";
            }
        }
        out_value << std::endl;
    }
    out_value.close();

    std::ofstream out_action;
    out_action.open("output/" + solver_type + "_action.csv", std::ios::out);
    out_action << std::setiosflags(std::ios::fixed) << std::setprecision(2);
    for (int i = 0; i < model->xw_cnt; ++i)
    {
        // xw_idx = xk*w_cnt + wk
        out_action << model->x_set.list[i/n_w] << ";";
        out_action << model->x_set.list[i%n_w] << ",";
    }
    out_action << std::endl;
    for (int k = 0; k < N; k++)
    {
        for (int xk = 0; xk < n_x; ++xk)
        {
            for (int wk = 0; wk < n_w; ++wk)
            {
                int idx = k*(n_x*n_w) + xk*n_w + wk;
                out_action << a[idx] << ",";
            }
                
        }
        out_action << std::endl;
    }
    out_action.close();

    return 0;
}