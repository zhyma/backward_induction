#ifndef UTILITY_H_
#define UTILITY_H_

#include <string>
#include <fstream>
#include <iomanip>

template<typename T>
int mat_to_file(std::string file_name, int dim_len, long *dim, T *mat);

int result_to_file(std::string solver_type, int *dim, float *v, int * a);

class DataLoader
{
    public:
        DataLoader(std::string filename);
        ~DataLoader();
        int read_state(float &dc, int &intention);
        int next_trial();
        bool end_of_file = false;
        float param[4]= {};
        std::string f_name;

    private:
        std::ifstream in_file;
        
};

class DataWriter
{
    public:
        DataWriter(std::string filename);
        ~DataWriter();
        template<typename T>
        int write(T data);
    
    private:
        std::string filename;
        std::ofstream out_file;
};

#endif //UTILITY_H_