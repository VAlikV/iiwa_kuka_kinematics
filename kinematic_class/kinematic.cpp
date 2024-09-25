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

bool Kinematic::compareVectors(vec a, vec b)
{
    for (int i = 0; i < size(a)(0); ++i)
    {
        if (abs((a(i)-b(i))) >= 0.01)
        {
            return false;
        }
    }
    return true;
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

vec Kinematic::getEndefectorOrientation()
{
    return endefector_pose_;
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

mat Kinematic::Rx(float const thetta)
{
    mat R = {{1, 0, 0},
            {0, cos(thetta), -sin(thetta)},
            {0, sin(thetta), cos(thetta)}};
    return R; 
}

mat Kinematic::Ry(float const thetta)
{
    mat R = {{cos(thetta), 0, sin(thetta)},
            {0, 1, 0},
            {-sin(thetta), 0, cos(thetta)}};
    return R; 
}

mat Kinematic::Rz(float const thetta)
{
    mat R = {{cos(thetta), -sin(thetta), 0},
            {sin(thetta), cos(thetta), 0},
            {0, 0, 1}};
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

    this->eulerZYZ(temp);
}

// ---------------------------------------------------------------- Обратная кинематика

void Kinematic::eulerZYZ(const mat& rot_mat)
{
    vec angles(3);

    angles(0) = (rot_mat(0,2));
    angles(1) = (rot_mat(1,2));
    angles(2) = (rot_mat(2,2));

    // mat E(3,3,fill::eye);
    // angles(0) = atan2(rot_mat(1,2), rot_mat(0,2));
    // angles(1) = atan2(sqrt(1-rot_mat(2,2)*rot_mat(2,2)), rot_mat(2,2));
    // angles(2) = atan2(rot_mat(2,1), -rot_mat(2,0));

    // double c = rot_mat(2,2);
    // if (c > -1 && c < 1) {
    //     double s = sqrt(1-c*c);
    //     angles(0) = atan2(rot_mat(1,2),rot_mat(0,2)); 
    //     angles(1) = atan2(c,s);
    //     angles(2) = atan2(rot_mat(2,1),-rot_mat(2,0));   
    // }   
    // else if(c == 1) {
    //     angles(0) = atan2(rot_mat(1,0),rot_mat(0,0));   // theta + psy
    //     angles(1) = 0;
    //     angles(2) = 0; }
    // else {
    //     angles(0) = atan2(-rot_mat(0,1),-rot_mat(0,0)); // theta-psy
    //     angles(1) = datum::pi;
    //     angles(2) = 0;
    // }

    endefector_pose_ = angles;
    // endefector_pose_ = vec({0, 0, angles(0)}) + this->Rz(angles(0))*vec({0, angles(1), 0}) + this->Rz(angles(0))*this->Ry(angles(1))*vec({0, 0, angles(2)});
    // endefector_pose_ = {(rot_mat(0,2)), rot_mat(1,2), rot_mat(0,1)};
}

vec Kinematic::z_i_1(int joint)
{
    mat z_i(3,3,fill::eye);

    for(int i = 0; i < joint; ++i)
    {
        z_i *= this->R(alpha_[i],thetta_[i]);
    }

    return {z_i(0,2), z_i(1,2), z_i(2,2)};
}

vec Kinematic::p_i_1(int joint)
{
    mat p_i(4,4,fill::eye);

    for(int i = 0; i < joint; ++i)
    {
        p_i *= this->T(alpha_[i], d_[i], a_[i], thetta_[i]);
    }

    return {p_i(0,3), p_i(1,3), p_i(2,3)};
}

mat Kinematic::Jacobian()
{
    this->FK();
    mat J(6,N_JOINTS);
    vec z_i;
    vec p_i;

    for(int i = 0; i < N_JOINTS; ++i)
    {
        z_i = this->z_i_1(i);
        p_i = cross(z_i, endefector_coordinate_ - this->p_i_1(i));
        J.col(i) = join_cols(p_i,z_i);
        // J.col(i) = join_cols(z_i,p_i);
    }
    return J;
}

void Kinematic::IK(const vec target_pos)
{
    mat J(6,N_JOINTS);
    vec end_coord(3);
    vec zero = {0,0,0};

    for(int i = 0; i < 1000000; ++i)
    {
        J = Jacobian();
        end_coord = join_cols(endefector_coordinate_, endefector_pose_);
        if (this->compareVectors(target_pos, end_coord))
        {
            std::cout << "Itera: " << i << std::endl;
            break;
        }
        thetta_ = thetta_ + pinv(J) * (target_pos - end_coord);
    }
}