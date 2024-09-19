#include "kinematic_class/kinematic.hpp"
#include "usable_functions/usable_functions.hpp"

using namespace iiwa_kunematic;

int main(int argc, char** argv)
{
    Kinematic iiwa = Kinematic();
    
    iiwa.setThettaDeg({0, 60, -60, 70, 40, 10, 0});
    // iiwa.setThettaDeg({0, 0, 0, 0, 0, 0, 0});
    iiwa.FK();
    mat coord = iiwa.getJointsCoordinates();

    saveMat("joints.txt", coord);

    return 0;
}