#include "../APP/Variable.hpp"

TD tar_pitch(150);
TD tar_yaw(150);
TD tar_yaw_vel(150);
TD tar_pitch_vel(150);

TD tar_shoot(30);

TD Yaw_vel(100);

TD Yaw_out(0);

TD shoot_vel_Left(100);
TD shoot_vel_Right(100);

Kpid_t Kpid_yaw_angle(0, 0, 0.0);
Kpid_t Kpid_yaw_vel(0.0, 0.0, 0.0);

PID pid_yaw_angle(0, 0);
PID pid_yaw_vel(0.0, 0.5);

Kpid_t Kpid_pitch_angle(40.0, 0.0, 0.0);
Kpid_t Kpid_pitch_vel(0.23, 0.0, 0.0);

PID pid_pitch_angle(30.0, 2.0);
PID pid_pitch_vel(0.0, 0.0);

Ude yaw_ude(15, 0.12, 150, 100);

Kpid_t Kpid_Friction_L_vel(0, 0, 0);
Kpid_t Kpid_Friction_R_vel(0, 0, 0);
PID pid_Friction_L_vel(0.0, 0.0);
PID pid_Friction_R_vel(0.0, 0.0);

Alg::LADRC::Adrc Adrc_yaw_vel(Alg::LADRC::TDquadratic(100, 0.004), 4, 0.6, 20.0, 20.0, 0.004, 3.5);
Alg::LADRC::Adrc Adrc_pitch_vel(Alg::LADRC::TDquadratic(100, 0.004), 6, 0.9, 25, 30.0, 0.004, 3.5);

Alg::LADRC::Adrc Adrc_Friction_L(Alg::LADRC::TDquadratic(250, 0.002), 15.0, 0.1, 27, 1.2, 0.002, 16384);
Alg::LADRC::Adrc Adrc_Friction_R(Alg::LADRC::TDquadratic(250, 0.002), 15.0, 0.1, 27, 1.2, 0.002, 16384);

Kpid_t Kpid_Dail_pos(0, 0, 0);
Kpid_t Kpid_Dail_vel(200, 0, 0);

PID pid_Dail_pos(10, 0); 
PID pid_Dail_vel(0, 0);