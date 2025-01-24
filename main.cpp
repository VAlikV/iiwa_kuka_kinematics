#include <time.h>
#include <iostream>
#include <Eigen/Dense>
#include "facade/main_kinematic.hpp"

// using namespace iiwa_kunematic;

int main(int argc, char** argv)
{
    Eigen::Matrix<double,3,3> rotation;
    Eigen::Array<double,3,1> position;

    Eigen::Array<double, N_JOINTS,1> q;
    q << 0*M_PI/180, 
        75*M_PI/180, 
        90*M_PI/180, 
        -40*M_PI/180, 
        -20*M_PI/180,
        30*M_PI/180,
        10*M_PI/180;

    Eigen::Array<double, N_JOINTS,1> q_ik;
    q_ik << 0*M_PI/180, 
        100*M_PI/180, 
        75*M_PI/180, 
        -25*M_PI/180, 
        -40*M_PI/180,
        10*M_PI/180,
        0*M_PI/180;

    Eigen::Array<double,3,1> position_ik(0.861414, 0.224974, 0.132113);
    Eigen::Matrix<double,3,3> rotation_ik;
    rotation_ik << -0.602578,  -0.16522,   0.78077,
                    0.425703,   0.76098,  0.489578,
                    -0.675038,  0.627385, -0.388216;

    int ik_result;
    clock_t t;
    double time_taken;


    std::cout << "Hello, I'm kinematics solvers test!" << std::endl << std::endl;

    std::cout << "Angles for tests: " << q.transpose() << std::endl << std::endl; 

    std::cout << "Position for tests: " << position_ik.transpose() << std::endl << std::endl; 
    std::cout << "Matrix for tests: " << rotation_ik << std::endl << std::endl; 

    // ---------------------------------------------------------- Инициализация KDL

    std::cout << "===========================================================" << std::endl;

    std::cout << "1) KDL solver" << std::endl << std::endl;

    KDLKinematic* KDL_solver = new KDLKinematic();

    Kinematic kinematic_KDL = Kinematic(KDL_solver);

    kinematic_KDL.setQRad(q);
    kinematic_KDL.FK();

    position = kinematic_KDL.getPositionVector();
    rotation = kinematic_KDL.getRotationMatrix();

    std::cout << "Position: " << position.transpose() << std::endl;
    std::cout << "Matrix: " << rotation << std::endl << std::endl;

    t = clock();

    kinematic_KDL.setPositionVector(position_ik);
    kinematic_KDL.setRotationMatrix(rotation_ik);

    ik_result = kinematic_KDL.IK();

    q_ik = kinematic_KDL.getQDeg();

    t = clock() - t;

    time_taken = ((double)t)/CLOCKS_PER_SEC;
    std::cout << "Время расчета: " << time_taken*1000 << std::endl;

    kinematic_KDL.setQDeg(q_ik);
    kinematic_KDL.FK();

    position = kinematic_KDL.getPositionVector();
    rotation = kinematic_KDL.getRotationMatrix();

    std::cout << "Result: " << ik_result << std::endl;
    std::cout << "Angles: " << q_ik.transpose() << std::endl;
    std::cout << "IK position: " << position.transpose() << std::endl;
    std::cout << "IK rotation: " << rotation << std::endl << std::endl;

    // ---------------------------------------------------------- Инициализация SNS

    std::cout << "===========================================================" << std::endl;

    std::cout << "2) SNS solver" << std::endl;

    SNSKinematic* SNS_solver = new SNSKinematic(sns_ik::VelocitySolveType::SNS);

    Kinematic kinematic_SNS = Kinematic(SNS_solver);

    // ---------------------------------------------------------- Инициализация Pinocchio

    std::cout << "\n\n===========================================================" << std::endl;

    std::cout << "3) Pinocchio solver" << std::endl << std::endl;

    PinKinematic* Pin_solver = new PinKinematic("../robots/iiwa.urdf");

    Kinematic kinematic_Pin = Kinematic(Pin_solver);

    kinematic_Pin.setQRad(q);
    kinematic_Pin.FK();

    position = kinematic_Pin.getPositionVector();
    rotation = kinematic_Pin.getRotationMatrix();

    std::cout << "Position: " << position.transpose() << std::endl;
    std::cout << "Matrix: " << rotation << std::endl << std::endl;

    t = clock();

    kinematic_Pin.setPositionVector(position_ik);
    kinematic_Pin.setRotationMatrix(rotation_ik);

    ik_result = kinematic_Pin.IK();

    q_ik = kinematic_Pin.getQDeg();

    t = clock() - t;

    time_taken = ((double)t)/CLOCKS_PER_SEC;
    std::cout << "Время расчета: " << time_taken*1000 << std::endl;

    kinematic_Pin.setQDeg(q_ik);
    kinematic_Pin.FK();

    position = kinematic_Pin.getPositionVector();
    rotation = kinematic_Pin.getRotationMatrix();

    std::cout << "Result: " << ik_result << std::endl;
    std::cout << "Angles: " << q_ik.transpose() << std::endl;
    std::cout << "IK position: " << position.transpose() << std::endl;
    std::cout << "IK rotation: " << rotation << std::endl << std::endl;

    // ---------------------------------------------------------- Инициализация Drake

    std::cout << "===========================================================" << std::endl;

    std::cout << "4) Drake solver" << std::endl << std::endl;

    DrakeKinematic* Drake_solver = new DrakeKinematic("../robots/iiwa.urdf");

    Kinematic kinematic_Drake = Kinematic(Drake_solver);

    kinematic_Drake.setQRad(q);
    kinematic_Drake.FK();

    position = kinematic_Drake.getPositionVector();
    rotation = kinematic_Drake.getRotationMatrix();

    std::cout << "\nPosition: " << position.transpose() << std::endl;
    std::cout << "Matrix: " << rotation << std::endl << std::endl;

    t = clock();

    kinematic_Drake.setPositionVector(position_ik);
    kinematic_Drake.setRotationMatrix(rotation_ik);

    ik_result = kinematic_Drake.IK();

    q_ik = kinematic_Drake.getQDeg();

    t = clock() - t;

    time_taken = ((double)t)/CLOCKS_PER_SEC;
    std::cout << "Время расчета: " << time_taken*1000 << std::endl;

    kinematic_Drake.setQDeg(q_ik);
    kinematic_Drake.FK();

    position = kinematic_Drake.getPositionVector();
    rotation = kinematic_Drake.getRotationMatrix();

    std::cout << "Result: " << ik_result << std::endl;
    std::cout << "Angles: " << q_ik.transpose() << std::endl;
    std::cout << "IK position: " << position.transpose() << std::endl;
    std::cout << "IK rotation: " << rotation << std::endl << std::endl;


    // ---------------------------------------------------------- Задание углов

    
    return 0;
}