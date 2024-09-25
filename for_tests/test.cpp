#include <armadillo>

using namespace arma;

int main(int argc, char** argv)
{

    mat A(3,7);

    // A(span(0,1), span(0,1)).print();

    A.col(0).print();

    return 0;
}