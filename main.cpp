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

    // Eigen::Array<double,N_JOINTS,3> a = kinematic.getJointPose();
    // std::cout << a << "\n\n";

    Eigen::Matrix<double,3,3> rotation;

    rotation << -1, 0, 0,
                0, 1, 0,
                0, 0, -1;

    Eigen::Array<double,3,1> position;

    Eigen::Array<double,N_JOINTS,1> q;

    double pose[] {0,0,0};

    while (true)
    {
        std::cin >> pose[0] >> pose[1] >> pose[2];
        position << pose[0]/100, pose[1]/100, pose[2]/100;

        clock_t t = clock();
        int state = kinematic.IK(rotation, position);
        t = clock() - t;
        double time_taken = ((double)t)/CLOCKS_PER_SEC;

        q = kinematic.getQDeg();
        kinematic.FK();
        
        std::cout << "Статус: " << state << std::endl;
        std::cout << "Время расчета: " << time_taken*1000 << std::endl;

        Eigen::Array<double,3,1> j = kinematic.getPositionVector();
        Eigen::Array<double,3,3> i = kinematic.getRotationMatrix();

        std::cout << std::setprecision(4) << "Положение: " << std::endl << (double)j(0)*100 << " " << (double)j(1)*100 << " " << (double)j(2)*100 << std::endl;
        std::cout << std::setprecision(4) << "Матрица: " << std::endl << i << std::endl << std::endl;
    }

    // position << -0.45, 0.45, 0.3;

    // Eigen::Array<double,N_JOINTS,3> b = kinematic.getJointPose();
    // std::cout << b << std::endl;

    // saveMatrix("joints.txt",a);
    // saveMatrix("joints1.txt",b);

    return 0;
}