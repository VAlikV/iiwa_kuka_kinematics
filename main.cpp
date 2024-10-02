// #include "kinematic_class/kinematic.hpp"
#include <time.h>
#include <iostream>
#include <Eigen/Dense>
// #include "kinematic_class/KDL/KDL_kinematic.hpp"
#include "facade/main_kinematic.hpp"

// using namespace iiwa_kunematic;

int main(int argc, char** argv)
{
    std::cout << "Hello, I'm kinematics solver!\n\n";

    KDLKinematic* KDL_solver = new KDLKinematic();

    Kinematic kinematic = Kinematic(KDL_solver);

    Eigen::Array<double,N_JOINTS,3> a = kinematic.getJointPose();

    std::cout << a << "\n\n";



    Eigen::Matrix<double,3,3> rotation;

    rotation << -1, 0, 0,
                0, 1, 0,
                0, 0, -1;

    // KDL::Rotation 
    // rotation = kinematic.getRotationMatrix();

    Eigen::Array<double,3,1> position;

    position << 0.41, 0.41, 0.5; 

    // position = kinematic.getPositionVector();

    // Eigen::Array<double,N_JOINTS,1> Q = kinematic.getQRad();
    // Q << 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57; 
    // kinematic.setQRad(Q);
    // kinematic.FK();

    clock_t t = clock();
    int state = kinematic.IK(rotation, position);
    kinematic.FK();
    t = clock() - t;
    double time_taken = ((double)t)/CLOCKS_PER_SEC;
    std::cout << "Время расчета: " << time_taken*1000 << std::endl;

    Eigen::Array<double,N_JOINTS,1> j = kinematic.getQDeg();

    std::cout << "Углы: " << j << std::endl;
    std::cout << "Статус: " << state << std::endl;

    Eigen::Array<double,N_JOINTS,3> b = kinematic.getJointPose();

    std::cout << b << std::endl;

    saveMatrix("joints.txt",a);
    saveMatrix("joints1.txt",b);


    return 0;
}