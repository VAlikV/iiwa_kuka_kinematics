#include "kinematic_class/kinematic.hpp"

using namespace std;
using namespace iiwa_kunematic;

int main(int argc, char** argv)
{
    Kinematic iiwa = Kinematic();
    
    // iiwa.setThettaDeg({0, 30, 45, -30, 20, 10, 0});
    iiwa.setThettaDeg({0, 0, 0, 0, 0, 0, 0});
    iiwa.FK();
    mat coord = iiwa.getJointsCoordinates();
    coord.print();
    return 0;
}