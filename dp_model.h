#ifndef DP_MODEL_H_
#define DP_SOLVER_H_

#include <iostream>
#include <cmath>
#include <math.h>
#include <fstream>
#include <stdlib.h>
#include <time.h>

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
        int prob_type;
        bool GPU = false;

        bool save_transition;
        int iter;

        Set x_set;
        Set u_set;
        Set w_set;

        int xw_cnt;
        int states_cnt;
        
        float *value_table;
        int *action_table;

        DPModel();

    private:
        // Running the forward search once, you will get a
        float *temp_search;
        // A counter for all next states that appears. The extra one (last one, [x_cnt]) is for the sum of one current state.
        int *cnter_table;
        // The final model you get. The probability of transiting from one state to another.
        float *prob_table;

        float *center_distribution;
        
        int discretize(Set *in);

        int val_to_idx(float val, Set *ref);
        int xw_idx(int xk, int wk);
        int state_idx(int k, int xk, int wk);
        int sas2idx(int xk, int wk, int uk, int xk_, int wk_);

};
#endif // DP_SOLVER_H_