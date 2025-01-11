#include "Pinocchio_kinematic.hpp"

PinKinematic::PinKinematic(sns_ik::VelocitySolveType type)
{
    
}

PinKinematic::~PinKinematic()
{
    
}

// ----------------------------------------------------------------------- Сетеры/Гетеры

void PinKinematic::setQ(const Eigen::Array<double,N_JOINTS,1> &thetta)
{
    thetta_.data = thetta;
}

Eigen::Array<double,N_JOINTS,1> PinKinematic::getQ()
{
    return thetta_.data;
}

// -----------------------

void PinKinematic::setNullBias(const Eigen::Array<double,N_JOINTS,1> &thetta)
{

}

Eigen::Array<double,N_JOINTS,1> PinKinematic::getNullBias()
{
    Eigen::Array<double,N_JOINTS,1> nulb;
    return nulb;
}

// -----------------------

void PinKinematic::setPositionVector(const Eigen::Vector3d &position)
{
    endefector_.p.x(position(0));
    endefector_.p.y(position(1));
    endefector_.p.z(position(2));
}

Eigen::Vector3d PinKinematic::getPositionVector()
{
    Eigen::Vector3d pos;
    pos << endefector_.p.x(), endefector_.p.y(), endefector_.p.z();
    return pos;
}

// -----------------------

void PinKinematic::setRotationMatrix(const Eigen::Matrix<double,3,3> &rotation)
{
    // KDL::Rotation rot = KDL::Rotation(rotation(0,0), rotation(0,1), rotation(0,2), 
    //                 rotation(1,0), rotation(1,1), rotation(1,2), 
    //                 rotation(2,0), rotation(2,1), rotation(2,2));
    // endefector_.M = rot;
}

Eigen::Matrix<double,3,3> PinKinematic::getRotationMatrix()
{
    // Eigen::Matrix<double,3,3> rot;
    // rot << endefector_.M.data[0], endefector_.M.data[1], endefector_.M.data[2],
    //         endefector_.M.data[3], endefector_.M.data[4], endefector_.M.data[5],
    //         endefector_.M.data[6], endefector_.M.data[7], endefector_.M.data[8];
    // return rot;
}

// -----------------------

Eigen::Matrix<double,N_JOINTS,3> PinKinematic::getJointPose()
{
    Eigen::Matrix<double,N_JOINTS,3> joint_pose;
    
    return joint_pose;
}

// ----------------------------------------------------------------------- Кинематика

int PinKinematic::FK()
{
    
}

// bool KDLKinematic::FK(const Eigen::Array<double,N_JOINTS,1> &thetta)
// {
//     this->setQ(thetta);
//     return this->FK();
// }

int PinKinematic::IK()
{
    
}

// bool KDLKinematic::IK(const Eigen::Matrix<double,3,3> &rotation, const Eigen::Array<double,3,1> &position)
// {
//     this->setRotationMatrix(rotation);
//     this->setPositionVector(position);
//     return this->IK();
// }

