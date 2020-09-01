#ifndef PHY_MODEL_H_
#define PHY_MODEL_H_

#include <iostream>
#include <time.h>
#include <stdlib.h>
using namespace std;

float disturb(int k);
float linear_model(int k, float x, float u);

#endif // PHY_MODEL_H_