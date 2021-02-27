#ifndef SIMULATE_H_
#define SIMULATE_H_

#include <ctime>
#include <string>
#include <sstream>

#include "dp_model.h"
#include "cpu_solver.h"
#include "gpu_solver.h"
#include "utility.h"

#define CPU_SOLVER 0
#define GPU_SOLVER 1
#define COMPARE    2

float one_iter(int solver, bool log, DPModel * dp_model);
int run_iters(int iters, int solver, DPModel * dp_model);
int compare(CPUSolver * c_solver, GPUSolver * g_solver);

#endif