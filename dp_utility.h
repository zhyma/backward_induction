#ifndef DP_UTILITY_H_
#define DP_UTILITY_H_

struct Min_index
{
    int index;
    float value;
};

// move to physical model
typedef struct Set
{
    int count;
    float *list;
    float bound[2];
} Set;

typedef struct Transition
{
    int xk;
    int wk;
    float p;
    int xk_;
    int wk_;
} Trans;

#endif // DP_UTILITY_H_