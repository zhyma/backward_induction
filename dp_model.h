#ifndef DP_MODEL_H_
#define DP_MODEL_H_

#define PENALTY 1e15

#include <iostream>
#include <cmath>
// #include <math.h>
// #include <stdlib.h>
// #include <time.h>

#include <dirent.h>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>

#include <thread>
#include <atomic>

#include "tinyxml2/tinyxml2.h"

using namespace tinyxml2;

// move to physical model
typedef struct Set
{
    long n;
    float *list;
    float min;
    float max;
} Set;

class DPModel
{
    public:

        bool debug = false;
        bool new_prob = false;
        int iter;
        // predict 10 steps forward
        int N_pred = 10;
        // run the controller 10 times
        int N_run = 10;
        int N_total = N_pred + N_run - 1;
        // the number of sampling point along the ego car's trajectory
        int n_d = 128;
        int n_v = 32;
        int n_a = 32;
        //  the number of sampling point along the front car's trajectory
        int n_dc = 185;
        int max_last_step = 13;
        int n_p = 28;
        int n_p_gpu = 32;

        Set d;
        Set v;
        Set a;

        Set x;
        Set u;
        Set w;

        // Save cost-to-go as a matrix
        float *r_cost;
        long *r_mask;
        // Save terminal cost as a matrix
        float *t_cost;
        
        // save <x,w> -u-> x'
        long *s_trans_table;
        float *prob_table;

        DPModel(int pred_steps, int running_steps);
        ~DPModel();
        // int terminal_cost_init(float d0);
        // long terminal_cost(int dk0, int dk, int vk);
        float terminal_cost(long xk, long wk);
        int get_dist_idx(float dist);
        int get_velc_idx(float velc);
        int get_subset(int k0, int dk0, int dck0);
        int get_subset_gpu(int k0, int dk0, int dck0);
        int phy_model(float *attr, float ax);

        int action_filter(float v0, int a_idx);

    private:
        float dt=2;
        // distance to the traffic light
        float d2tl;
        // the time that the red light will start
        int rl_start;
        // the time that the red light will end
        int rl_end;
        float t_ttc;
        // weight of the car
        float m = 1500;
        float g = 9.8;
        int phy_model(int xk, int uk);
        int discretize(Set *in);
        int state_trans();
        int running_cost_init();

        //
        bool front_car_safe(float dx, float vx, float dcx);
        bool red_light_safe(int k, float dx, float vx, float ax);

        // if you have multiple probability matrices
        float* p_mat;
        int check_driving_data();

        int val_to_idx(float val, Set *ref);
        int copy_p_mat();

        bool test_set = false;

};
#endif // DP_MODEL_H_

