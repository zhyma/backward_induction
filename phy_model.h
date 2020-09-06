#ifndef PHY_MODEL_H_
#define PHY_MODEL_H_

#include <iostream>
#include <time.h>
#include <stdlib.h>
#include <fstream>
#include <math.h>
using namespace std;

float fix_disturb(int k);
float mc_disturb(int k);
float linear_model(int k, float x, float u);

#endif // PHY_MODEL_H_