#ifndef DP_MODEL_H_
#define DP_MODEL_H_

#include <iostream>
#include <cmath>
// #include <math.h>
// #include <stdlib.h>
// #include <time.h>

#include <thread>
#include <atomic>

#include "phy_model.h"

// move to physical model
typedef struct Set
{
    int count;
    float *list;
    float bound[2];
} Set;

class DPModel
{
    public:
        PHYModel * ptr_model;

        bool save_transition;
        int iter;
        int N = 10;

        int sample_trials = 10e5;
        int sample_size;

        Set x_set;
        Set u_set;
        Set w_set;

        // Save cost-to-go as a matrix
        float *cost2go;
        // Save terminal cost as a matrix
        float *t_cost;

        int xw_cnt;
        int states_cnt;
        
        // save <x,w> -u-> x'
        int *s_trans_table;
        float *prob_table[2];
        // float *value_table;
        // int *action_table;

        DPModel(PHYModel * ptr_in, int steps, int x_grain, int w_grain, int u_grain, std::atomic<int>* busy_p_mat);
        int daemon(std::atomic<bool>* running);

    private:
        std::atomic<int>* busy_mat_ptr;
        int discretize(Set *in);
        int state_trans();
        int cost_init();

        float *p_mat_temp;
        int distribution();
        int gen_w_trans_mat(int update_mat, int prob_type);

        int val_to_idx(float val, Set *ref);
        int xw_idx(int xk, int wk);
        int state_idx(int k, int xk, int wk);
        int sas2idx(int xk, int wk, int uk, int xk_, int wk_);

};
#endif // DP_MODEL_H_