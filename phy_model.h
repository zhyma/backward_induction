#ifndef PHY_MODEL_H_
#define PHY_MODEL_H_

#define NO_DISTURB 0
#define MC_DISTURB 1
#define FIX_DISTURB 2

#include <iostream>
#include <time.h>
#include <stdlib.h>
#include <fstream>
#include <math.h>
#include <random>
using namespace std;

class PHYModel
{
    public:
        int disturb_type;
        int N = 10;
        float sigma = 1;
        float x_bound[2] = {-2.0, 2.0};
        float u_bound[2] = {0.2, 1.6};
        //constraint for w is {0, 2}
        float w_bound[2] = {0.0, 2.0};
        
        float w_0;

        PHYModel(int disturb_selector, float s);
        float linear_model(int k, float x, float u, float prev_w);

    private:
        std::random_device rd;
        std::mt19937 gen;
        float gaussian(float in);
        float fix_disturb(int k);
        float mc_disturb(float last_w);
};


#endif // PHY_MODEL_H_