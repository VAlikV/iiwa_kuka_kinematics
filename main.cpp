#include <iostream>
#include <armadillo>

using namespace std;
using namespace arma;

int main(int argc, char** argv)
{
    cout << "Armadillo version: " << arma_version::as_string() << endl;

    mat A(3,3);  // directly specify the matrix size (elements are uninitialised)

    cout << "A.n_rows: " << A.n_rows << endl;  // .n_rows and .n_cols are read only
    cout << "A.n_cols: " << A.n_cols << endl;

    A(1,1) = 456.0;  // directly access an element (indexing starts at 0)
    A.print("A:");

    mat v(3,3);
    v << 1 << 2 << 3 << endr << 4 << 5 << 6 << endr << 7 << 8 << 9 << endr;
    v.print("v:");

    mat c;
    c = A % v;
    c.print("c: ");

    return 0;
}