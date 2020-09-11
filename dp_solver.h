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
        int solve_one_step(int k);
        int solve_whole_model();

    private:
        int find_min(float *u, int cnt, struct Min_index *min);
};


#endif // DP_SOLVER_H_