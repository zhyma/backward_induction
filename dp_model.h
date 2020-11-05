#ifndef DP_MODEL_H_
#define DP_MODEL_H_

#include <iostream>
#include <cmath>
#include <math.h>
#include <fstream>
#include <stdlib.h>
#include <time.h>

#include "phy_model.h"
#include "prob_tool.h"

struct Min_index
{
    int index;
    float value;
};

// move to physical model
typedef struct Set
{
    int count;
    float *list;
    float bound[2];
} Set;

typedef struct Transition
{
    int xk;
    int wk;
    float p;
    int xk_;
    int wk_;
} Trans;

class DPModel
{
    public:
        PHYModel * ptr_model;
        bool GPU = false;

        bool save_transition;
        int iter;
        int N = 10;
        int grain;

        Set x_set;
        Set u_set;
        Set w_set;

        int xw_cnt;
        int states_cnt;
        
        // save <x,w> -u-> x'
        int *s_trans_table;
        float *prob_table;
        float *value_table;
        int *action_table;

        DPModel(PHYModel * ptr_in, int grain);

    private:
        // Running the forward search once, you will get a
        float *temp_search;
        // A counter for all next states that appears. The extra one (last one, [x_cnt]) is for the sum of one current state.
        int *cnter_table;

        float *center_distribution;
        
        int discretize(Set *in);
        int state_trans();

        int val_to_idx(float val, Set *ref);
        int xw_idx(int xk, int wk);
        int state_idx(int k, int xk, int wk);
        int sas2idx(int xk, int wk, int uk, int xk_, int wk_);

};
#endif // DP_MODEL_H_