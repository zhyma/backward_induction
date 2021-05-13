#ifndef FRONT_CAR_H
#define FRONT_CAR_H
#include <random>
#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>

int fc_n_step_sim(int iter);

class FCar
{
    public:
        float d2tl = 0;
        float dt = 2;
        float v_bound[2] = {0, 18.0};
        float a_bound[2] = {-4, 2};
        
        float d = 0;
        float v = 0;
        float a = 0;
        int intention = 1;
        
        FCar(float d2tl_in, float dt_in, float * v_bound_in, float * a_in);
        int sim_step();
        int vehicle_ctrl(bool rl);

    private:
        float draw_a_number(float lower, float upper);
        int make_decision(float percentage);
        
        std::random_device rd;
        std::mt19937 gen;
};

#endif