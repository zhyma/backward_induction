#ifndef GPU_SOLVER_H_
#define GPU_SOLVER_H_

#include "dp_model.h"
#include "utility.h"

template <typename T1>
class BIData
{
	public:
		T1 *cpu;
		T1 *gpu;
		int dimc; //count of dimensions
		int *dimv; // value of dimensions
		int size;
		int size_b;

		template <typename... T2>
		int init(T2... args)
		{   
            // number of dimensions 
			dimc = sizeof...(args);
			size = 1;
            // size of each dimension
			dimv = new int[dimc]{args...};
			for (int i = 0; i < dimc; ++i)
				size = size * dimv[i];
		    // std::cout << size << std::endl;
		    size_b = size * sizeof(T1);
		    // std::cout << size_b << std::endl;

            cpu = new T1[size]{};
            return 0;
		};

        ~BIData()
        {
            delete [] cpu;
            delete [] dimv;
        }
};

class GPUSolver
{
    public:
        GPUSolver(DPModel * ptr_in, int block_size_in);
        ~GPUSolver();

        DPModel * model;
        int N;
        int n_x, n_w, n_u;
        int n_x_s, n_w_s;

        bool debug = false;

        int solve(int k0, float d0, float v0, float dc0, int intention);
        // float * value;
        // int * action;
        BIData<float> value;
        BIData<int> action;

    private:
        int block_size;
        int n_v;

        int n_p, n_p_default;
        int n_d;

        BIData<float> r_cost;
        BIData<unsigned long long int> r_mask;
        BIData<int> trans;
        BIData<float> prob;
        BIData<float> q;

        // find_min and calc_q are provided by gpu_kernel.h
        // estimate_one_step is integrated into solve()
        int get_subset(int k0, int dk0, int dck0);
};



#endif //GPU_SOLVER_H_