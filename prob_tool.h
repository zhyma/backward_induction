#ifndef PROB_TOOL_H_
#define PROB_TOOL_H_

#include <random>
#include <string.h>
#include <iostream>

class Prob_gen
{
    public:
        int trials;
        int state_cnt; // N_w, also known as the number of bins
        float sigma;
        float bound[2];
        float *prob_mat; // N_w by N_w

        Prob_gen(int number_of_trials, int number_of_states, float given_sigma, float *state_boundary);
        int gen_trans_mat();

    private:
        float *mat_temp;
        int norm_disturb();
};

#endif  // PROB_TOOL_H_