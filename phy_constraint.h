// struct Phy_constraint
// {
//     int N=10;
//     float x[2] = {-2.0, 2.0};
//     float u[2] = {0.2, 1.6};
//     //constraint for w is {0, 2}
//     float w[2] = {0.0, 2.0};
// }bounds;

#define N 10
#define x_lower -2.0
#define x_upper 2.0
#define u_lower 0.2
#define u_upper 1.6
#define w_lower 0.0
#define w_upper 2.0

#define grain 1000