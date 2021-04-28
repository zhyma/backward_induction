#include <iostream>
#include <fstream>
#include <sstream>

#include <string>
#include <cstring>

#include "simulate.h"
#include "front_car.h"
#include "dp_model.h"
#include "tinyxml2/tinyxml2.h"

std::string get_param(XMLElement* elmt_root, const char* tag)
{
    const char* param_char = elmt_root->FirstChildElement(tag)->GetText();
    std::string param_str(param_char);
    // remove '\r', '\n', ' ' from the string
    param_str.erase(0, param_str.find_first_not_of(" \r\n"));
    param_str.erase(param_str.find_first_of(" \r\n"));
    // std::cout << "Read: " << param_str << std::endl;
    return param_str;
}

int main(int argc, char *argv[])
{
    int pred_steps = 4;

    tinyxml2::XMLDocument doc_xml;
    //XMLError err_xml = doc_xml.LoadFile("../config.xml");
    XMLError err_xml = doc_xml.LoadFile("config.xml");

    if(XML_SUCCESS==err_xml)
    {
        XMLElement* elmt_root = doc_xml.RootElement();
        std::string step_str = get_param(elmt_root, "prediction_steps");
        pred_steps = std::stoi(step_str);
        std::cout << "prediction steps is set to " << pred_steps << std::endl;
    }
    else
        std::cout << "config.xml read error" << std::endl;

    std::string modeStr = "";
    if (argc > 1)
    {
        modeStr.assign(argv[1], strlen(argv[1]));
    }

    if (modeStr == "one_step" || modeStr == "n_step")
    {
        int run_steps = 1;

        DPModel dp_model(pred_steps, run_steps);
        if (modeStr == "one_step")
        {
            // run one step
            std::cout << "run simulation for one step" << std::endl;
            if (argc > 2)
            {
                std::string solverStr = "";
                solverStr.assign(argv[2], strlen(argv[2]));
                if (solverStr == "cpu")
                    one_step(CPU_SOLVER, true, &dp_model);
                else if(solverStr == "gpu")
                    one_step(GPU_SOLVER, false, &dp_model);
            }
            else
            {
                dp_model.new_prob = true;
                one_step(CPU_SOLVER, false, &dp_model);
            }
        }
        // else if (modeStr == "n_step")
        // { 
        //     // whole simulation
        //     std::cout << "run simulation for n steps" << std::endl;
        //     int pred_steps = 10;
        //     // int run_steps = 10;
        //     int run_steps = 10;
        //     int trials = 1;

        //     run_trials(trials, run_steps, CPU_SOLVER, &dp_model);
        // }
    }
    
    // else if (modeStr == "gen_data")
    // {
    //     // generate front car data
    //     std::cout << "generate front car driving data" << std::endl;
    //     int iter = 10;
    //     if (argc > 2)
    //     {
    //         std::string val(argv[2]); 
    //         iter = std::stoi(val);
    //         if (iter <= 0)
    //             iter = 10;
    //     }
    //     std::cout << "Simulate the front car for " << iter << " times" << std::endl;
        
    //     fc_n_step_sim(iter);
    // }

    // std::cout << "done" << std::endl;

    return 0;
}