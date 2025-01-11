#ifndef PIN_KINEMATIC_HPP
#define PIN_KINEMATIC_HPP

#include <Eigen/Dense>

#include "../base_kinematic.hpp"
#include <vector>

using namespace iiwa_kinematics;

class PinKinematic: public BaseKinematic
{
private: 

    // --------------------------------------------- Для инициализации

    

    // --------------------------------------------- Расчеты

    

public:
    PinKinematic();
    ~PinKinematic();

    void setQ(const Eigen::Array<double,N_JOINTS,1> &thetta) override;
    Eigen::Array<double,N_JOINTS,1> getQ() override;

    void setNullBias(const Eigen::Array<double,N_JOINTS,1> &thetta) override;
    Eigen::Array<double,N_JOINTS,1> getNullBias() override;

    void setRotationMatrix(const Eigen::Matrix<double,3,3> &rotation) override;
    Eigen::Matrix<double,3,3> getRotationMatrix() override;

    void setPositionVector(const Eigen::Vector3d &position) override;
    Eigen::Vector3d getPositionVector() override;

    Eigen::Matrix<double,N_JOINTS,3> getJointPose();

    int FK() override;
    // bool FK(const Eigen::Array<double,N_JOINTS,1> &thetta) override;

    int IK() override;
    // bool IK(const Eigen::Matrix<double,3,3> &rotation, const Eigen::Array<double,3,1> &position) override;
    
};

#endif