#ifndef PHY_MODEL_H_
#define PHY_MODEL_H_

#define NO_DISTURB 0
#define MC_DISTURB 1
#define FIX_DISTURB 2

struct Phy_constraint
{
    int N = 10;
    float sigma = 1;
    float x_bound[2] = {-2.0, 2.0};
    float u_bound[2] = {0.2, 1.6};
    //constraint for w is {0, 2}
    float w_bound[2] = {0.0, 2.0};
};



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

        int linear_model(int k, float x, float w, float u, float * next);
    //private:
        
        
};


#endif // PHY_MODEL_H_