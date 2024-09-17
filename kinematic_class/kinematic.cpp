#include "kinematic.hpp"

Kinematic::Kinematic()
{
    thetta_ = zeros(7);
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

void Kinematic::setThetta(vec thetta)
{
    thetta_ = this->deg2rad(thetta);
}

// ---------------------------------------------------------------- Прямая кинематика

mat Kinematic::T(float const alpha, float const d, float const a, float const thetta)
{
    mat T = {{cos(thetta), -sin(thetta)*cos(alpha), sin(thetta)*sin(alpha), a*cos(thetta)}, {sin(thetta), cos(thetta)*cos(alpha), -cos(thetta)*sin(alpha), a*sin(thetta)}, {0, sin(alpha), cos(alpha), d}, {0, 0, 0, 1}};
    return T;
}

