#ifndef DP_MODEL_H_
#define DP_MODEL_H_

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
    int n;
    float *list;
    float min;
    float max;
} Set;

class DPModel
{
    public:

        bool debug = false;
        int iter;
        // predict 10 steps forward
        int N_pred = 10;
        // run the controller 10 times
        int N_run = 10;
        int N_total = N_pred + N_run - 1;
        int n_t = 128;
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
        long *r_cost;
        unsigned long long int *r_mask;
        // Save terminal cost as a matrix
        long *t_cost;
        
        // save <x,w> -u-> x'
        int *s_trans_table;
        float *prob_table;

        DPModel(int pred_steps, int running_steps);
        ~DPModel();
        // int terminal_cost_init(float d0);
        long terminal_cost(int dk0, int dk, int vk);
        int get_dist_idx(float dist);
        int get_velc_idx(float velc);
        int get_subset(int k0, int dk0, int dck0);
        int get_subset_gpu(int k0, int dk0, int dck0);
        int phy_model(float *attr, float ax);

        int action_filter(float v0, int a_idx);

    private:
        float dt=2;
        // distance to the traffic light
        int d2tl;
        // the time that the red light will start
        int rl_start;
        // the time that the red light will end
        int rl_end;
        float t_tcc;
        // weight of the car
        float m = 1500;
        float g = 9.8;
        int phy_model(int xk, int uk);
        int discretize(Set *in);
        int state_trans();
        int running_cost_init();

        //
        bool front_car_safe(float dx, float vx, float dcx);

        // if you have multiple probability matrices
        float* p_mat;
        int check_driving_data();

        int val_to_idx(float val, Set *ref);
        int copy_p_mat();

};
#endif // DP_MODEL_H_

