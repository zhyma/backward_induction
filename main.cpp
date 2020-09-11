#include "phy_model.h"
#include "forward_search.h"
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
        string disturb_type = get_param(elmt_root, "disturb_type");
        int noise_type = NO_NOISE;
        if (disturb_type.compare("markov") == 0)
        {
            noise_type = MC_NOISE;
            cout << "config to Markov Chain disturbance\n";
        }
        else if (disturb_type.compare("fix") == 0)
        {
            noise_type = FIX_NOISE;
            cout << "config to fixed disturbance\n";
        }       
        else
        {
            noise_type = NO_NOISE;
            cout << "config to no disturbance\n";
        }

        //Initializing the model
        PHYModel phy_model(noise_type);
        DPModel dp_model(&phy_model, gran, false);
        DPSolver solver(&dp_model);

        string solver_type = get_param(elmt_root, "solver_type");
        //solving method: get the whole model, save to memory, then use DP to search backward at once.
        if (solver_type.compare("whole_model")==0)
        {
            cout << "estimate the whole model first" << endl;
            dp_model.estimate_model(100);
            cout << "move on to solver" << endl;
            start = clock();
            solver.solve_whole_model();
            end = clock();
        }
        
        
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