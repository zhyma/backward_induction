#include "dp_model.h"

int write_to_file(DPModel * model, float *v, int * a)
{
    std::ofstream out_value;
    out_value.open("value.csv", std::ios::out);
    //title needs to be re-assigned
    for (int i = 0; i < model->xw_cnt; ++i)
    {
        // xw_idx = xk*w_cnt + wk
        out_value << model->x_set.list[i/model->w_set.count] << ";";
        out_value << model->x_set.list[i%model->w_set.count] << ",";
    }
    out_value << std::endl;

    for (int k = 0; k < model->N+1; k++)
    {
        for (int xk = 0; xk < model->x_set.count; ++xk)
        {
            for (int wk = 0; wk < model->w_set.count; ++wk)
            {
                int idx = k*(model->x_set.count*model->w_set.count) + xk*model->w_set.count + wk;
                out_value << v[idx] << ",";
            }
        }
        out_value << std::endl;
    }
    out_value.close();

    std::ofstream out_action;
    out_action.open("action.csv", std::ios::out);
    for (int i = 0; i < model->xw_cnt; ++i)
    {
        // xw_idx = xk*w_cnt + wk
        out_action << model->x_set.list[i/model->w_set.count] << ";";
        out_action << model->x_set.list[i%model->w_set.count] << ",";
    }
    out_action << std::endl;
    for (int k = 0; k < model->N; k++)
    {
        for (int xk = 0; xk < model->x_set.count; ++xk)
        {
            for (int wk = 0; wk < model->w_set.count; ++wk)
            {
                int idx = k*(model->x_set.count*model->w_set.count) + xk*model->w_set.count + wk;
                out_action << a[idx] << ",";
            }
                
        }
        out_action << std::endl;
    }
    out_action.close();
    
    return 0;
}