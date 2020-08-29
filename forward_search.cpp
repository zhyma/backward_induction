#include "forward_search.h"
#include "model.h"

struct StateProb
{
    int state_idx;  // the s' for <s,a> pair
    float prob;     // the probability of ending up at that s'
};

class DPModel
{
    public:
        int N = 10;                     // time steps
        float x_con[2] = {-2, 2};       // "x_con[]" is the constraints of x.
        float u_con[2] = {0.2, 1.6};    // "u_con[]" is the constraints of u.
        int gran = 10;                  // granularity.

        int x_cnt = (int)(round((x_con[1]-x_con[0])*gran+1));
        int u_cnt = (int)(round((u_con[1]-u_con[0])*gran+1));

        float *temp_search= new float[N * x_cnt * u_cnt];
        

        int kxu2index(int k, float x, float u)
        {
            int idx = k * (x_cnt * u_cnt);
            idx += x * u_cnt + u;
            return idx;
        }

        // int index2kxu()
        // {
        //     return 0;
        // }

        // Search forward one time. Once you search for multiple times, you can get a "grid" model for DP
        // Return a tensor. The index is a tuple <k, x_k, a>, the value corresponding to the tuple is the x_{k+1}
        int forward_search_once(float x0)
        {
            float delta = 1.0/gran;
            int idx = 0;
            for(int k = 0; k < N; k++)
            {
                int x_cnter = 0;
                for(float xk = x_con[0]; xk < x_con[1] + delta; xk += delta)
                {
                    int u_cnter = 0;
                    for(float uk=u_con[0]; uk < u_con[1] + delta; uk += delta)
                    {
                        float x = linear_model(k, xk, uk);
                        idx = kxu2index(k, x_cnter, u_cnter);
                        temp_search[idx] = x;
                        u_cnter += 1;
                    }
                    x_cnter += 1;
                }
            }
            return 0;
        }

        int estimate_model()
        {

        }

};

int main()
{
    DPModel model;
    model.forward_search_once(0.8);
    cout << "========================================\n";
    for(int i = 0; i < model.N * model.x_cnt * model.u_cnt; i++)
    {
        cout << model.temp_search[i] << endl;
    }
    return 0;
}