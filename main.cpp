#include "kinematic_class/kinematic.hpp"

using namespace std;

int main(int argc, char** argv)
{
    Kinematic iiwa = Kinematic();
    vec a = {1,2,3,4,5,6,7};
    iiwa.setThetta(a);
    iiwa.setThetta({2,2,2,2,2,2,2});
    iiwa.printDH();

    mat b = iiwa.T(0,0,0,datum::pi/2);
    b.print("b: ");

    vec c = iiwa.deg2rad({90});
    c.print("c: ");

    return 0;
}