#include "iostream"
using namespace std;

def value_iteration(P, nS, nA, V, gamma=0.9, tol=1e-8):

    
    V_new = V.copy()
    ############################
    # YOUR IMPLEMENTATION HERE #
    while True:
        delta = 0
        for s in range(nS):
            v = V_new[s]
            q = np.zeros(nA)
            for a in range(nA):
                # sum[p*(r+gamma*Vs')]
                q[a] = sum([i[0]*(i[2]+gamma*V_new[i[1]]) for i in P[s][a]])
            V_new[s] = np.max(q)
            delta = np.max([delta, abs(v-V_new[s])])

        if delta < tol:
            break

    print(V_new)

    policy_new = np.zeros([nS, nA])
    for s in range(nS):
        v = V_new[s]
        act = np.zeros(nA)
        for a in range(nA):
            # sum[p*(r+gamma*Vs')]
            act[a] = sum([i[0] * (i[2] + gamma * V_new[i[1]]) for i in P[s][a]])

        policy_new[s][np.argmax(act)] = 1

    print(policy_new)
    ############################
    return policy_new, V_new

struct NewPolicy{
    int *policy_new;
    double *V_new;
}

/*Learn value function and policy by using value iteration method for a given
gamma and environment.

Parameters:
----------
P, nS, nA, gamma:
    defined at beginning of file
V: value to be updated
tol: float
    Terminate value iteration when
        max |value_function(s) - prev_value_function(s)| < tol
Returns:
----------
policy_new: np.ndarray[nS,nA]
V_new: np.ndarray[nS]*/
struct NewPolicy value_iteration(double *P, nS, nA, V, double gamma, double tol)
{
    //make a backup of current value table
    for(int i = 0; i < strlen(V);i++)
    {

    }
    //create a q table for state-action pairs
    for(int i=0;)
    //to get value table converaged
    while(true)
    {
        double delta=0;
        for(int i = 0;i < nS;i++)
        {
            for(int j = 0;j < nA;j++)
            {

            }
        }
        if(delta < tol)
        {
            break;
        }
    }


    //extract policy

    return policy_new, 
}