%% THIS PROGRAM IS USED TO DESIGN THE PID CONTROLLERS FOR THE QUADROTOR

clc
clear all
close all

%% DYNAMICS



%% GAINS

Kp_phi = 0; Ki_phi = 0; Kd_phi = 0;
Kp_th  = 0; Ki_th  = 0; Kd_th  = 0;
Kp_psi = 0; Ki_psi = 0; Kd_psi = 0;
Kp_T   = 0; Ki_T   = 0; Kd_T   = 0;

%% RLTOOL
