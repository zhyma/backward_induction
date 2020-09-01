#ifndef FORWARD_SEARCH_H_
#define FORWARD_SEARCH_H_

#include <iostream>
#include <cmath>
#include <fstream>
#include "phy_model.h"
using namespace std;

class DPModel
{
    public:
        int N;                          // time steps
        float x_con[2];                 // "x_con[]" is the constraints of x.
        float u_con[2];                 // "u_con[]" is the constraints of u.
        int gran;                       // granularity.

        // total number of x and u (after discretized)
        int x_cnt;
        int u_cnt;

        // all possible x and u
        float *x_list;
        float *u_list;

        // Running the forward search once, you will get a 
        float *temp_search;
        // A counter for all next states that appears. The extra one (last one, [x_cnt]) is for the sum of one current state.
        int *cnter_table;
        // The final model you get. The probability of transiting from one state to another.
        float *prob_table;

        DPModel(int time_step, float * state_con, float * action_con, int sample_rate);
        int kxu2index(int k, int x, int u);
        int x2x_cnt(float x);
        int forward_search_once(float x0);
        int estimate_model(int i);

};
#endif // FORWARD_SEARCH_H_