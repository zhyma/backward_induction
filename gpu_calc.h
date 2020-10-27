#include "dp_utility.h"
void terminal_value(Set x_set, Set w_set, float *input, float *output);
void intermediate_value(Set x_set, Set w_set, float *prob, float *value, float *output);
void gpu_backward_induction(Set x_set, Set w_set, Set u_set, Trans *trans_table, float *value_table, int *action_table);