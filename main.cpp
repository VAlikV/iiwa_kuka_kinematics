#include "kinematic_class/kinematic.hpp"
#include "usable_functions/usable_functions.hpp"
#include <time.h>

using namespace iiwa_kunematic;

int main(int argc, char** argv)
{
    Kinematic iiwa = Kinematic();
    
    iiwa.setThettaDeg({30, 60, 0, -30, 0, 90, 20});
    // iiwa.setThettaDeg({0, 0, 0, 0, 0, 0, 0});
    iiwa.FK();
    
    mat coord = iiwa.getJointsCoordinates();
    saveMat("joints.txt", coord);

    vec ik_coor = iiwa.getEndefectorCoordinates();
    vec ik_pose = iiwa.getEndefectorOrientation();

    // vec ik_coor = {250, 467, 250};
    // vec ik_pose = {-0.5, 0.6, -0.62};
    
    ik_coor.print("1");
    ik_pose.print("1");

    ik_coor = join_cols(ik_coor, ik_pose);
    // vec zero = {0,0,0};
    // ik_coor = join_cols(ik_coor, zero);

    iiwa.setThettaDeg({10, 10, 10, 10, 10, 10, 10});
    iiwa.FK();
    
    clock_t t = clock();
    iiwa.IK(ik_coor);
    t = clock() - t;
    double time_taken = ((double)t)/CLOCKS_PER_SEC;
    std::cout << time_taken*1000 << std::endl;

    ik_coor= iiwa.getEndefectorCoordinates();
    ik_pose = iiwa.getEndefectorOrientation();

    ik_coor.print("2");
    ik_pose.print("2");
    
    mat coord1 = iiwa.getJointsCoordinates();

    saveMat("joints1.txt", coord1);

    return 0;
}