#ifndef DP_SOLVER_H_
#define DP_SOLVER_H_

#define NO_NOISE 0
#define MC_NOISE 1
#define FIX_NOISE 2

#include "phy_model.h"
#include "forward_search.h"

#include <iostream>
#include <time.h>
#include <stdlib.h>
#include <fstream>
#include <math.h>
#include <time.h>
using namespace std;

struct Min_index
{
    int index;
    float value;
};

class DPSolver
{
    public:
        DPModel * ptr_model;
        float *value_table;
        float *action_table;

        DPSolver(DPModel * ptr_in);
        float solve_one_step(int k);
        int solve_whole_model();
        int write_to_file();


    private:
        float *temp_search;
        int *cnter_table;
        float *prob_table;

        int find_min(float *u, int cnt, struct Min_index *min);
        int xu2index(int xk, int uk, int x_);
        int search_one_step(int k, int iter);
};


#endif // DP_SOLVER_H_