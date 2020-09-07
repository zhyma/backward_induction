#ifndef PHY_MODEL_H_
#define PHY_MODEL_H_

#define NO_NOISE 0
#define MC_NOISE 1
#define FIX_NOISE 2

#include <iostream>
#include <time.h>
#include <stdlib.h>
#include <fstream>
#include <math.h>
using namespace std;

class PHYModel
{
    public:
        int disturb_type;
        int N = 10;
        float x_bound[2] = {-2.0, 2.0};
        float u_bound[2] = {0.2, 1.6};
        //constraint for w is {0, 2}

        PHYModel(int disturb_selector);
        float fix_disturb(int k);
        float mc_disturb(float w);
        float linear_model(int k, float x, float u, float last_w);
};


#endif // PHY_MODEL_H_