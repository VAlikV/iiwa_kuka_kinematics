#include <armadillo>

using namespace arma;

int main(int argc, char** argv)
{

    mat A(3,3);

    A(span(0,1), span(0,1)).print();

    return 0;
}