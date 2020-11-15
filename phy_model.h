#ifndef PHY_MODEL_H_
#define PHY_MODEL_H_

#include <random>

class PHYModel
{
    public:
        int disturb_type;
        // int N = 10;
        float sigma = 2.0/6.0;
        float x_bound[2] = {-2.0, 2.0};
        float u_bound[2] = {0.2, 1.6};
        //constraint for w is {0, 2}
        float w_bound[2] = {0.0, 2.0};
        
        //should be the distribution model of w_0
        float w_0;

        //PHYModel();
        float linear_model(float x, float w, float u);
        float next_w(float w);

    private:
        std::random_device rd;
        std::mt19937 gen;
        
};


#endif // PHY_MODEL_H_