#include "kinematic.hpp"
using namespace iiwa_kunematic;

Kinematic::Kinematic()
{
    thetta_ = zeros(7);

    endefector_coordinate_ = zeros(3);
    endefector_pose_ = zeros(3);

    joints_coordinate_ = zeros(3,7);

    this->FK();
}

// ---------------------------------------------------------------- Вспомогательное

void Kinematic::printDH()
{
    thetta_.print("thetta: ");
}

vec Kinematic::deg2rad(vec degrees)
{
    return degrees*datum::pi/180;
}


vec Kinematic::rad2deg(vec rads)
{
    return rads*180/datum::pi;
}

// ---------------------------------------------------------------- Сетеры/гетеры

void Kinematic::setThettaDeg(vec thetta)
{
    thetta_ = this->deg2rad(thetta);
}

void Kinematic::setThettaRad(vec thetta)
{
    thetta_ = thetta;
}

mat Kinematic::getJointsCoordinates()
{
    return joints_coordinate_;
}

vec Kinematic::getEndefectorCoordinates()
{
    return endefector_coordinate_;
}

// ---------------------------------------------------------------- Прямая кинематика

mat Kinematic::T(float const alpha, float const d, float const a, float const thetta)
{
    mat T = {{cos(thetta), -sin(thetta)*cos(alpha), sin(thetta)*sin(alpha), a*cos(thetta)},
            {sin(thetta), cos(thetta)*cos(alpha), -cos(thetta)*sin(alpha), a*sin(thetta)},
            {0, sin(alpha), cos(alpha), d}, {0, 0, 0, 1}};
    return T;
}

mat Kinematic::R(float const alpha, float const thetta)
{
    mat R = {{cos(thetta), -sin(thetta)*cos(alpha), sin(thetta)*sin(alpha)},
            {sin(thetta), cos(thetta)*cos(alpha), -cos(thetta)*sin(alpha)},
            {0, sin(alpha), cos(alpha)}};
    return R;
}

void Kinematic::FK()
{
    mat temp(4,4,fill::eye); 

    for(int i = 0; i < N_JOINTS; ++i)
    {
        temp *= this->T(alpha_[i], d_[i], a_[i], thetta_[i]);
        joints_coordinate_.col(i) = {temp(0,3), temp(1,3), temp(2,3)};
        // temp.print(std::to_string(i));
    }

    endefector_coordinate_ = joints_coordinate_.col(N_JOINTS-1);

    /*
        Добавить ориентацию эндефектора
    */
}
