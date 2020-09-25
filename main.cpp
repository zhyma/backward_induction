#include "phy_model.h"
#include "dp_solver.h"

#include "tinyxml2/tinyxml2.h"
#include <iostream>
#include <cmath>
#include <fstream>
#include <sstream>
#include <time.h>

using namespace std;
using namespace tinyxml2;

string get_param(XMLElement* elmt_root, const char* tag)
{
    const char* param_char = elmt_root->FirstChildElement(tag)->GetText();
    string param_str(param_char);
    // remove '\r', '\n', ' ' from the string
    param_str.erase(0, param_str.find_first_not_of(" \r\n"));
    param_str.erase(param_str.find_first_of(" \r\n"));
    cout << "Read: " << param_str << endl;
    return param_str;
}

int main()
{
    tinyxml2::XMLDocument doc_xml;
    //XMLError err_xml = doc_xml.LoadFile("../config.xml");
    XMLError err_xml = doc_xml.LoadFile("config.xml");
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
        string disturb_str = get_param(elmt_root, "disturb_type");
        int disturb_type = NO_NOISE;
        if (disturb_str.compare("markov") == 0)
        {
            disturb_type = MC_NOISE;
            cout << "config to Markov Chain disturbance\n";
        }
        else if (disturb_str.compare("fix") == 0)
        {
            disturb_type = FIX_NOISE;
            cout << "config to fixed disturbance\n";
        }       
        else
        {
            disturb_type = NO_NOISE;
            cout << "config to no disturbance\n";
        }

        //Initializing the model
        PHYModel phy_model(disturb_type, 2.0/6.0);
        cout << "model ready" << endl;

        string prob_str = get_param(elmt_root, "probability_method");
        int prob_type = MONTECARLO;
        if (prob_str.compare("algebraic")==0)
        {
            int prob_type = ALGEBRAIC;
            cout << "TODO" << endl;
        }

        DPSolver solver(&phy_model, prob_type, gran, 1000);
        float total_time = 0;

        string solver_type = get_param(elmt_root, "solver_type");
        //solving method: get the whole model, save to memory, then use DP to search backward at once.
        if (solver_type.compare("whole_model")==0)
        {
            cout << "function no longer exist" << endl;
        }
        else if (solver_type.compare("one_step")==0)
        {
            float cost_time = 0;
            for (int k = phy_model.N; k >=0; k--)
            {
                cost_time = solver.solve_one_step(k);
                cout << "at k=" << k << ", spend " << cost_time << endl;
                total_time += cost_time;
            }
            solver.write_to_file();
        }
        
        cout << "total time cost: " << total_time << endl;
        cout << "done";
        return 0;
    }
    else
    {
        cout << "config.xml read error" << endl;
        return 0;
    }
}