#include "Pinocchio_kinematic.hpp"

PinKinematic::PinKinematic(std::string urdf_name)
{
    pinocchio::urdf::buildModel(urdf_name, model_);
    data_ = pinocchio::Data(model_);
    for (int i = 0; i < N_JOINTS; ++i)
    {
        thetta_max_[i] = LIMITS_MAX[i]*M_PI/180;
        thetta_min_[i] = LIMITS_MIN[i]*M_PI/180;

        thetta_previous_[i] = INIT[i];
        thetta_[i] = INIT[i];
        // thetta_previous_(i) = 0.785;
        // thetta_(i) = 0.785;
        // thetta_previous_(i) = 1.57;
        // thetta_ (i) = 1.57;
        // thetta_previous_(i) = 0;
        // thetta_(i) = 0;
    }

    this->FK();
}

PinKinematic::~PinKinematic()
{
    
}

// ----------------------------------------------------------------------- Сетеры/Гетеры

void PinKinematic::setQ(const Eigen::Array<double,N_JOINTS,1> &thetta)
{
    thetta_ = thetta.matrix();
}

Eigen::Array<double,N_JOINTS,1> PinKinematic::getQ()
{
    return thetta_.array();
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
    endefector_.translation() = position;
}

Eigen::Vector3d PinKinematic::getPositionVector()
{
    return endefector_.translation();
}

// -----------------------

void PinKinematic::setRotationMatrix(const Eigen::Matrix<double,3,3> &rotation)
{
    endefector_.rotation() = rotation;
}

Eigen::Matrix<double,3,3> PinKinematic::getRotationMatrix()
{
    return endefector_.rotation();
}

// -----------------------

Eigen::Matrix<double,N_JOINTS,3> PinKinematic::getJointPose()
{
    Eigen::Matrix<double,N_JOINTS,3> joint_pose;

    for (int i = 0; i < N_JOINTS; ++i)
    {
        joint_pose.row(i) = data_.oMi[i+1].translation();
    }
    
    return joint_pose;
}

// ----------------------------------------------------------------------- Кинематика

int PinKinematic::FK()
{
    thetta_previous_ = thetta_;
    pinocchio::forwardKinematics(model_, data_, thetta_);
    endefector_ = data_.oMi[7];
    return 1;
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

