#ifndef PHY_MODEL_H_
#define PHY_MODEL_H_

#include "phy_constraint.h"

__host__ __device__ float shared_linear_model(float x, float w, float u);

#endif // PHY_MODEL_H_