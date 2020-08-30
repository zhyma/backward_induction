#ifndef FORWARD_SEARCH_H_
#define FORWARD_SEARCH_H_

#include <iostream>
#include <cmath>
#include <fstream>
#include "model.h"
using namespace std;

class DPModel
{
    public:
        int N = 10;                     // time steps
        float x_con[2] = {-2, 2};       // "x_con[]" is the constraints of x.
        float u_con[2] = {0.2, 1.6};    // "u_con[]" is the constraints of u.
        int gran = 10;                  // granularity.

        // total number of x and u (after discretized)
        int x_cnt = (int)(round((x_con[1]-x_con[0])*gran+1));
        int u_cnt = (int)(round((u_con[1]-u_con[0])*gran+1));

        // Running the forward search once, you will get a 
        float *temp_search = new float[(N * x_cnt) * u_cnt];
        // A counter for all next states that appears. The extra one (last one, [x_cnt]) is for the sum of one current state.
        int *cnter_table = new int[(N * x_cnt) * u_cnt * (x_cnt + 1)] ();
        // The final model you get. The probability of transiting from one state to another.
        float *prob_table = new float[(N * x_cnt) * u_cnt * x_cnt];

        int kxu2index(int k, int x, int u);
        int x2x_cnt(float x);
        int forward_search_once(float x0);
        int estimate_model(int i);

};
#endif // FORWARD_SEARCH_H_