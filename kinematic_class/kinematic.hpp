#ifndef KINEMATIC_HPP
#define KINEMATIC_HPP

#include <iostream>
#include <armadillo>
#include <math.h>
#include <string>

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
        vec thetta_;    // Обобщенные координаты - углы в джоинтах 
        
        const float thetta_limits_[N_JOINTS] = {165, 115, 165, 115, 165, 115};     // Пределы углов в джоинтах

        vec endefector_coordinate_;
        vec endefector_pose_;

        mat joints_coordinate_;

        // ---------------------------------------------------------------- Вспомогательное

        bool compareVectors(vec a, vec b);
        
        // ---------------------------------------------------------------- Обратная кинематика
        
        vec z_i_1(int joint);
        vec p_i_1(int joint);

    public:

        Kinematic();

        // ---------------------------------------------------------------- Вспомогательное

        void printDH();
        vec deg2rad(vec degrees);
        vec rad2deg(vec rads);

        // ---------------------------------------------------------------- Сетеры/гетеры

        void setThettaDeg(vec thetta = zeros(N_JOINTS)); // В градусах
        void setThettaRad(vec thetta = zeros(N_JOINTS)); // В радианах

        mat getJointsCoordinates();

        vec getEndefectorCoordinates();
        vec getEndefectorOrientation();

        // ---------------------------------------------------------------- Прямая кинематика

        mat T(float const alpha = 0, float const d = 0, float const a = 0, float const thetta = 0);
        mat R(float const alpha = 0, float const thetta = 0);

        mat Rx(float const thetta);
        mat Ry(float const thetta);
        mat Rz(float const thetta);


        void FK();

        // ---------------------------------------------------------------- Обратная кинематика

        void eulerZYZ(const mat& rot_mat);
        mat Jacobian();
        void IK(const vec target_pos);
    };
}

#endif