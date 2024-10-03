#ifndef BASE_KINEMATIC_HPP
#define BASE_KINEMATIC_HPP

#include <fstream>
#include <iostream>
#include <Eigen/Dense>

namespace iiwa_kinematics
{
    const int N_JOINTS = 7;
    const double LIMITS_MAX[N_JOINTS] = {168, 118, 168, 118, 168, 118, 173};
    const double LIMITS_MIN[N_JOINTS] = {-168, -118, -168, -118, -168, -118, -173};
    const double LINKS[N_JOINTS] = {0.34, 0, 0.4, 0, 0.4, 0, 0.126};
    const int MAX_ITER = 100;
    const double EPS = 1e-2;

    class BaseKinematic
    {
    public:

        // ----------------------------------------------------------------------- Сетеры/Гетеры

        virtual void setQ(const Eigen::Array<double,N_JOINTS,1> &thetta) = 0;
        virtual Eigen::Array<double,N_JOINTS,1> getQ() = 0;

        virtual void setRotationMatrix(const Eigen::Matrix<double,3,3> &rotation) = 0;
        virtual Eigen::Matrix<double,3,3> getRotationMatrix() = 0;

        virtual void setPositionVector(const Eigen::Array<double,3,1> &position) = 0;
        virtual Eigen::Array<double,3,1> getPositionVector() = 0;

        virtual Eigen::Matrix<double,N_JOINTS,3> getJointPose() = 0;    

        // ----------------------------------------------------------------------- Кинематика

        virtual int FK() = 0;
        // virtual bool FK(const Eigen::Array<double,N_JOINTS,1> &thetta) = 0;

        virtual int IK() = 0;
        // virtual bool IK(const Eigen::Matrix<double,3,3> &rotation, const Eigen::Array<double,3,1> &position) = 0;
    };

    void saveMatrix(std::string name, Eigen::Matrix<double,N_JOINTS,3> mat);
}

#endif