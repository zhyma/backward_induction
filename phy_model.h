#ifndef PHY_MODEL_H_
#define PHY_MODEL_H_

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
        
        //should be the distribution model of w_0
        float w_0;

        //PHYModel();
        float linear_model(float x, float w, float u);
        
};


#endif // PHY_MODEL_H_