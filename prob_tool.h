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
        float *mat_out; // N_w by N_w

        Prob_gen(int number_of_trials, int number_of_states, float given_sigma, float *state_boundary);
        int gen_trans_mat();

    private:
        float *mat_temp;
        int norm_disturb();
};

Prob_gen::Prob_gen(int number_of_trials, int number_of_states, float given_sigma, float *state_boundary)
{
    trials = number_of_trials;
    state_cnt = number_of_states;
    sigma = given_sigma;
    memcpy(bound, state_boundary, 2*sizeof(float));
    mat_temp = new float[state_cnt]{};
    mat_out = new float[state_cnt*state_cnt]{};
    gen_trans_mat();
}

int Prob_gen::norm_disturb()
{
    std::random_device rd;
    std::mt19937 gen;
    std::normal_distribution<float> d((bound[0] + bound[1])/2, sigma);
    float gran = (bound[1]-bound[0])/state_cnt;
    int *list = new int[state_cnt]{};
    for (int i = 0; i < trials; ++i)
    {
        // get random number with normal distribution using gen as random source
        float o = d(gen);
        if (o < bound[0] + gran/2)
        {
            list[0] += 1;
        }
        else if (o > bound[1] - gran/2)
        {
            list[state_cnt-1] += 1;
        }
        else
        {
            int no = 1 + (int) ((o - bound[0]-gran/2)/gran);
            list[no] += 1;
        }
    }
    for (int i = 0;i < state_cnt; ++i)
        mat_temp[i] = (float) list[i]/(float) trials;
    return 0;
}

int Prob_gen::gen_trans_mat()
{
    norm_disturb();
    for (int i = 0; i < state_cnt; ++i)
    {
        // given w_k
        for (int offset = 0; offset < state_cnt/2 + 1; ++offset)
        {
            // check j = i-0, i-1, ..., 0
            int j_l = i - offset;
            if (j_l < 0)
                mat_out[i*state_cnt+0] += mat_temp[state_cnt/2 - offset];
            else
                mat_out[i*state_cnt+j_l] = mat_temp[state_cnt/2 - offset];

            // check j = i+1, i+2, ..., state_cnt-1
            int j_r = i + offset;
            if (j_r > state_cnt-1)
                mat_out[i*state_cnt+(state_cnt-1)] += mat_temp[state_cnt/2 + offset];
            else
                mat_out[i*state_cnt+j_r] = mat_temp[state_cnt/2 + offset];
        }
    }
    return 0;
}

#endif  // PROB_TOOL_H_