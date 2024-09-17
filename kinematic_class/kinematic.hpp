#ifndef KINEMATIC_HPP
#define KINEMATIC_HPP

#include <iostream>
#include <armadillo>
#include <math.h>

using namespace arma;

class Kinematic
{
private:
    // ДХ параметры 
    const float alpha_[7] = {-datum::pi/2, datum::pi/2, datum::pi/2, -datum::pi/2, -datum::pi/2, datum::pi/2, 0};
    const float d_[7] = {340, 0, 400, 0, 400, 0, 126};
    const float a_[7] = {0, 0, 0, 0, 0, 0, 0};
    vec thetta_;    // Обобщенные координаты - углы в джоинтах 
    
    const float thetta_limits_[7] = {165, 115, 165, 115, 165, 115};     // Пределы углов в джоинтах



public:
    Kinematic();

    // ---------------------------------------------------------------- Вспомогательное

    void printDH();
    vec deg2rad(vec degrees);
    vec rad2deg(vec rads);

    // ---------------------------------------------------------------- Сетеры/гетеры

    void setThetta(vec thetta = zeros(7));

    // ---------------------------------------------------------------- Прямая кинематика

    mat T(float const alpha = 0, float const d = 0, float const a = 0, float const thetta = 0);

};

#endif