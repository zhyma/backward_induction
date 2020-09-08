#include "phy_model.h"
#include "forward_search.h"
#include "tinyxml2/tinyxml2.h"
#include <iostream>
#include <cmath>
#include <fstream>
#include <sstream>
#include <time.h>

using namespace std;
using namespace tinyxml2;

struct Min_index
{
    int index;
    float value;
};

int find_min(float *u, int cnt, struct Min_index *min)
{
    int index = 0;
    float value = u[0];
    for(int i = 0;i < cnt; ++i)
    {
        if(u[i] < value)
        {
            value = u[i];
            index = i;
        }
    }
    min->index = index;
    min->value = value;
    return 0;
}

// all possible x and u
int solver(DPModel &model)
{
    float *value_table = new float[(model.N+1)*model.x_cnt]{};
    float *action_table = new float[model.N*model.x_cnt]{};
    // calculate the termianl cost at N=10
    // initial value for V_N is V_N(x)=J_f(x), final cost
    // J_f(x) = (1-x_N)^2
    for(int x = 0; x < model.x_cnt; ++x)
    {
        float v = pow(1-model.x_list[x],2);
        value_table[model.N*model.x_cnt+x] = v;
    }
    // calculate the running cost
    // searching backward
    // a temporary buffer to save all the result of executing different u for a given xk
    float *u_z_temp = new float[model.u_cnt];
    for(int k = model.N-1; k >= 0; k--)
    {
        for(int xk = 0; xk < model.x_cnt; ++xk)
        {
            for(int uk = 0; uk < model.u_cnt; ++uk)
            {
                // for each <x, u> (q in RL): l(x,u)+\sum P(z|x,u)V(z)
                // l(x) = x^2+u^2
                float x = model.x_list[xk];
                float u = model.u_list[uk];
                float l = x*x + u*u;
                float sum = 0;
                // z, or x_/x'
                for(int x_ = 0; x_ < model.x_cnt; ++x_)
                {
                    //<k, x_k> --u_k--> <k+1,x_k+1>
                    int idx = model.kxu2index(k, xk, uk);
                    float p_z = model.prob_table[idx*model.x_cnt + x_];
                    float v_z = value_table[(k+1)*model.x_cnt + x_];
                    sum += p_z*v_z;
                }
                u_z_temp[uk] = l+sum;
            }
            // v = min[l(x,u)+\sum P(z|x,u)V(z)]
            // find the minimium now.
            Min_index min;
            find_min(u_z_temp, model.u_cnt, &min);
            value_table[k*model.x_cnt + xk] = min.value;
            action_table[k*model.x_cnt + xk] = model.u_list[min.index];
        }
    }
    if(true)
    {
        // Save value table and optimal action table to files
        ofstream out_value;
        out_value.open("../value.csv", ios::out);
        for (int i = 0; i < model.x_cnt; ++i)
            out_value << model.x_list[i] << ",";
        out_value << endl;
        for (int k = 0; k < model.N+1; k++)
        {
            for (int xk = 0; xk < model.x_cnt; ++xk)
            {
                out_value << value_table[k*model.x_cnt + xk] << ",";
            }
            out_value << endl;
        }
        out_value.close();

        ofstream out_action;
        out_action.open("../action.csv", ios::out);
        for (int i = 0; i < model.x_cnt; ++i)
            out_action << model.x_list[i] << ",";
        out_action << endl;
        for (int k = 0; k < model.N; k++)
        {
            for (int xk = 0; xk < model.x_cnt; ++xk)
            {
                out_action << action_table[k*model.x_cnt + xk] << ",";
            }
            out_action << endl;
        }
        out_action.close();
    }
    
    return 0;
}

int main()
{
    tinyxml2::XMLDocument doc_xml;
    XMLError err_xml = doc_xml.LoadFile("../config.xml");
    clock_t start,end;
    if(XML_SUCCESS==err_xml)
    {
        XMLElement* elmt_root = doc_xml.RootElement();

        // Get the granulairty within a unit (discretizing to get states)
        const char* gran_char = elmt_root->FirstChildElement("granularity")->GetText();
        stringstream strValue;
        int gran;
        strValue << gran_char;
        strValue >> gran;
        cout << "Granularity is set to: " << gran << endl;

        // Get the noise type
        const char* disturb_char = elmt_root->FirstChildElement("disturb_type")->GetText();
        string disturb_str(disturb_char);
        // remove '\r', '\n', ' ' from the string
        disturb_str.erase(0, disturb_str.find_first_not_of(" \r\n"));
        disturb_str.erase(disturb_str.find_first_of(" \r\n"));
        cout << disturb_str << endl;
        int noise_type = NO_NOISE;
        if (disturb_str.compare("markov") == 0)
        {
            noise_type = MC_NOISE;
            cout << "config to Markov Chain disturbance\n";
        }
        else if (disturb_str.compare("fix") == 0)
        {
            noise_type = FIX_NOISE;
            cout << "config to fixed disturbance\n";
        }       
        else
        {
            noise_type = NO_NOISE;
            cout << "config to no disturbance\n";
        }

        PHYModel phy_model(noise_type);
        DPModel model(&phy_model, gran, false);
        model.estimate_model(100);
        cout << "move on to solver" << endl;
        start = clock();
        solver(model);
        end = clock();
        cout << (double) (end-start)/CLOCKS_PER_SEC << endl;
        cout << "done";
        return 0;
    }
    else
    {
        cout << "config.xml read error" << endl;
        return 0;
    }
}