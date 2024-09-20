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
    
    vec ik_data = iiwa.getEndefectorCoordinates();
    vec zero = {0,0,0};
    ik_data = join_cols(ik_data, zero);

    iiwa.setThettaDeg({0, 0, 0, 0, 0, 0, 0});

    saveMat("joints.txt", coord);

    iiwa.IK(ik_data);
    
    mat coord1 = iiwa.getJointsCoordinates();

    saveMat("joints1.txt", coord1);

    return 0;
}