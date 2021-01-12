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

#include <thread>
#include <atomic>

// move to physical model
typedef struct Set
{
    int n;
    float *list;
    float bound[2];
} Set;

class DPModel
{
    public:

        bool save_transition;
        int iter;
        int N = 10;
        int no_of_p;

        Set d;
        Set v;
        Set a;

        Set x;
        Set u;
        Set w;

        // Save cost-to-go as a matrix
        float *running_cost;
        // Save terminal cost as a matrix
        float *t_cost;
        
        // save <x,w> -u-> x'
        int *s_trans_table;
        float *prob_table[2];
        // float *value_table;
        // int *action_table;

        DPModel(int steps, std::atomic<int>* busy_p_mat);
        int daemon(std::atomic<bool>* running);

    private:
        float dt=2;
        // distance to the traffic light
        int d2tl;
        // the time that the red light will start
        int rl_start;
        // the time that the red light will end
        int rl_end;
        float t_tcc;
        float d_target = 400;
        float v_target = 30;
        // weight of the car
        float m = 1500;
        float g = 9.8;
        std::atomic<int>* busy_mat_ptr;
        int phy_model(int xk, int wk, int uk);
        int discretize(Set *in);
        int state_trans();
        int cost_init();

        // if you have multiple probability matrices
        std::vector<float*> p_mat;
        int check_driving_data();

        int val_to_idx(float val, Set *ref);

};
#endif // DP_MODEL_H_