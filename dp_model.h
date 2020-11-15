#ifndef DP_MODEL_H_
#define DP_MODEL_H_

#include <iostream>
#include <cmath>
#include <math.h>
#include <fstream>
#include <stdlib.h>
#include <time.h>

#include "phy_model.h"
// #include "prob_tool.h"

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
        int grain;

        int sample_trials = 10e5;

        Set x_set;
        Set u_set;
        Set w_set;

        int xw_cnt;
        int states_cnt;
        
        // save <x,w> -u-> x'
        int *s_trans_table;
        float *prob_table;
        // float *value_table;
        // int *action_table;

        DPModel(PHYModel * ptr_in, int grain);

    private:
        int discretize(Set *in);
        int state_trans();

        float *p_mat_temp;
        int w_distribution();
        int gen_w_trans_mat();

        int val_to_idx(float val, Set *ref);
        int xw_idx(int xk, int wk);
        int state_idx(int k, int xk, int wk);
        int sas2idx(int xk, int wk, int uk, int xk_, int wk_);

};
#endif // DP_MODEL_H_