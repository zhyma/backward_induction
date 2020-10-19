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

#endif // DP_UTILITY_H_