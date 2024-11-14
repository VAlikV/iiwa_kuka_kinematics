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

    // KDLKinematic* KDL_solver = new KDLKinematic();
    SNSKinematic* SNS_solver = new SNSKinematic(sns_ik::VelocitySolveType::SNS);

    Kinematic kinematic = Kinematic(SNS_solver);

    // Eigen::Array<double,N_JOINTS,3> a = kinematic.getJointPose();
    // std::cout << a << "\n\n";

    Eigen::Matrix<double,3,3> rotation;

    rotation = kinematic.getRotationMatrix();

    // rotation << 0, 0, 0,
    //             0, 0, 0,
    //             0, 0, 0;

    Eigen::Array<double,3,1> position; 

    position = kinematic.getPositionVector();

    std::cout << std::endl;
    std::cout << "Начальное положение: " << std::endl << (double)position(0)*100 << " " << (double)position(1)*100 << " " << (double)position(2)*100 << std::endl;
    std::cout << std::setprecision(4) << "Начатльная матрица: " << std::endl << rotation << std::endl;

    Eigen::Array<double,N_JOINTS,1> q;

    double pose[] {0,0,0};
    double q1[] {0,0,0,0,0,0,0};

    std::cout << std::endl;
    // std::cin >> q1[0] >> q1[1] >> q1[2] >> q1[3] >> q1[4] >> q1[5] >> q1[6];
    // q << q1[0], q1[1], q1[2], q1[3], q1[4], q1[5], q1[6];

    std::cin >> pose[0] >> pose[1] >> pose[2];
    position << pose[0]/100, pose[1]/100, pose[2]/100;

    clock_t t = clock();
    int state = kinematic.IK(rotation, position);
    t = clock() - t;
    double time_taken = ((double)t)/CLOCKS_PER_SEC;
    std::cout << "Время расчета: " << time_taken*1000 << std::endl;

    // kinematic.setQDeg(q);
    // kinematic.FK();
    
    std::cout << std::endl << "Статус: " << state << std::endl;

    Eigen::Array<double,3,1> j = kinematic.getPositionVector();
    Eigen::Array<double,3,3> i = kinematic.getRotationMatrix();

    Eigen::Array<double,N_JOINTS,1> k = kinematic.getQDeg();

    std::cout << std::setprecision(4) << "Положение: " << std::endl << (double)j(0)*100 << " " << (double)j(1)*100 << " " << (double)j(2)*100 << std::endl;
    std::cout << std::setprecision(4) << "Матрица: " << std::endl << i << std::endl;
    std::cout << std::setprecision(4) << "Q: " << (double)k(0) << " " << (double)k(1) << " " << (double)k(2) << " " << (double)k(3) << " " << (double)k(4) << " " << (double)k(5) << " " << (double)k(6) << std::endl;

    // position << -0.45, 0.45, 0.3;

    // Eigen::Array<double,N_JOINTS,3> b = kinematic.getJointPose();
    // std::cout << b << std::endl;

    // saveMatrix("joints.txt",a);
    // saveMatrix("joints1.txt",b);

    return 0;
}