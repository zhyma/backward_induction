#include "iostream"
using namespace std;
#include <vector>


struct VTable{
    std::vector<int> policy_new;
    std::vector<float> V_new;
};

struct VTable backward_induction(float x0,int N)
{
    //make a backup of current value table
    for(int i = 0; i < strlen(V);i++)
    {

    }
    //create a q table for state-action pairs
    for(int i=0;i<10;i++)
    {}
    //to get value table converaged
    while(true)
    {
        double delta=0;
        //V_k=min(l(x,u)+\sum P(z|x,u)V_{k+1}(z))
        for(int i = N;i > 0;i++)
        {
            for(int j = 0;j < nA;j++)
            {
                
            }
        }
        if(delta < tol)
        {
            break;
        }
    }


    //extract policy

    return policy_new;
}

int main()
{

}