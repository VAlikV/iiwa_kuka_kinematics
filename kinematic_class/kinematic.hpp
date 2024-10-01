#ifndef KINEMATIC_HPP
#define KINEMATIC_HPP

#include <iostream>
#include <armadillo>
#include <math.h>
#include <string>
// #include "../usable_functions/usable_functions.hpp"

namespace iiwa_kunematic
{
    using namespace arma;

    const int N_JOINTS = 7;

    class Kinematic
    {
    private:
        // ДХ параметры 
        const float alpha_[N_JOINTS] = {-datum::pi/2, datum::pi/2, datum::pi/2, -datum::pi/2, -datum::pi/2, datum::pi/2, 0};
        const float d_[N_JOINTS] = {0.34, 0, 0.4, 0, 0.4, 0, 0.126};
        const float a_[N_JOINTS] = {0, 0, 0, 0, 0, 0, 0};
        vec thetta_;

        // mat T_;         // Матрица с подставленными параметрами ДХ
        
        const float thetta_limits_[N_JOINTS] = {165, 115, 165, 115, 165, 115};     // Пределы углов в джоинтах

        vec endefector_coordinate_;     // Координаты эндефектора
        vec endefector_pose_;           // Ориентация эндефектора

        mat joints_coordinate_;         // Координаты джоинтов

        mat z_i_;   // Для Якобиана
        mat p_i_;   // Для Якобиана

        // ---------------------------------------------------------------- Вспомогательное

        bool compareVectors(vec a, vec b);
        
        // ---------------------------------------------------------------- Обратная кинематика
        

    public:

        Kinematic();

        // ---------------------------------------------------------------- Вспомогательное

        void printDH();
        vec deg2rad(vec const degrees);
        vec rad2deg(vec const rads);

        // ---------------------------------------------------------------- Сетеры/гетеры

        void setThettaDeg(vec const thetta = zeros(N_JOINTS)); // В градусах
        void setThettaRad(vec const thetta = zeros(N_JOINTS)); // В радианах

        mat getJointsCoordinates();

        vec getEndefectorCoordinates();
        vec getEndefectorOrientation();

        // ---------------------------------------------------------------- Прямая кинематика

        mat T(float const alpha = 0, float const d = 0, float const a = 0, float const thetta = 0);
        // mat T(mat const T, float const thetta = 0);

        mat R(float const alpha = 0, float const thetta = 0);

        mat Rx(float const thetta);
        mat Ry(float const thetta);
        mat Rz(float const thetta);


        void FK();

        // ---------------------------------------------------------------- Обратная кинематика

        vec eulerZYZ(const mat& rot_mat);
        mat Jacobian();
        void IK(vec const target_pos);
    };
}

void saveMat(std::string name, iiwa_kunematic::mat matrix);

void saveVec(std::string name, iiwa_kunematic::vec vector);

#endif