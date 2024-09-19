#include "usable_functions.hpp"
using namespace iiwa_kunematic;

void saveMat(std::string name, mat matrix)
{
    std::ofstream ofs;
    ofs.open(name, std::ofstream::out | std::ofstream::trunc);

    if (!ofs.is_open()) { 
        std::cout << "ERROR" << std::endl;
        return;
    } 
  
    for (int i = 0; i < matrix.n_rows; ++i)
    {
        for (int j = 0; j < matrix.n_cols; ++j)
        {
            ofs << matrix(i,j) << "\t";
        } 
        ofs << "\n";
    } 
  
    ofs.close(); 
}