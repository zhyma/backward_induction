- Test under Windows 10 + VS code + CMake + Mingw-w64.
- If there is no error during compiling, but the executable file cannot be executed, try to uncomment the "#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -v -static-libstdc++")" in "CMakeLists.txt"
<br />

- There are three main parts in this repo:
    1. A very simple linear model (phy_model.cpp/phy_model.h)
    2. A module to search forward (forward_search.cpp/forward_search.h) using the given model (the linear model or an alternative one)
    3. A module to solve the stochastic dynamic programming (dp_solver.cpp)
<br />

- The forward_search.cpp search from time step k=0 to k=N. By giving the constraint of x and the granularity, discritizing x into states and search for all possible states. This module will generate a <S, A, R(cost), P, S'> and save into a table ("prob.csv"). P is generated by taking a 100 trials and get the (transition times)/(total number of trials), or Monte Carlo method.
<br />

- TODO (implement)
    1. "one step forward" (only search for the s->s' right in front of the step that used for calculating the value table).
    2. Without any search, from the given linear equation, derive the transition probability distribution.
<br />

- The probability model will be saved in "prob.csv"
- The value table is saved to "value.csv"
- The optimal action is saved to "action.csv"
- Use "show_value.py" to visualize the value table and the optimal actions.