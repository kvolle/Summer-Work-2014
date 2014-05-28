%% SYMBOLIC EQUATIONS OF MOTION OF THE QUADROTOR

clc
clear all
close all

syms phi th psi p q r u v w X Y Z k1 k2 dF1 dF2 dF3 dF4 l Jx Jy Jz g mu m real
syms T tau_phi tau_th tau_psi

%% EQUATION OF MOTION -  States - roll-pitch-yaw p-q-r x-y-z(Inertial) u-v-w(Body)

Rt = [1 sin(phi)*tan(th)  cos(phi)*tan(th); 0  cos(phi)  -sin(phi); 0  sin(phi)/cos(th)  cos(phi)/cos(th)];
 xdot(1:3,1) = Rt*[p;q;r];
 
 dF = [dF1;dF2;dF3;dF4];
 
%  FT = [k1 k1 k1 k1;0 -l*k1 0  l*k1; l*k1 0  -l*k1 0; -k2  k2  -k2  k2]*dF;
 
%  F       = FT(1);
%  tau_phi = FT(2);
%  tau_th  = FT(3);
%  tau_psi = FT(4);
FT(1) = T;
FT(2) = tau_phi;
FT(3) = tau_th;
FT(4) = tau_psi;
 xdot(4:6,1) = [((Jy-Jz)*q*r + tau_phi)/Jx; ((Jz-Jx)*p*r + tau_th)/Jy; ((Jx-Jy)*p*q + tau_psi)/Jz];
 
 xdot(7:9,1)   = (Ex(phi)*Ey(th)*Ez(psi))'*[u;v;w];
 
%  T  = F; % Thrust at CG along body z-axis
 xdot(10:12,1) = [-g*sin(th)+(v*r-w*q) - mu*u/m; g*sin(phi)*cos(th)+(w*p-u*r)-mu*v/m; g*cos(phi)*cos(th)+(u*q-v*p)-T/m];

 % ---------- EKF WITH KNOWN MU ----------------
 %% STATES TO BE ESTIMATED
 
 X    = [phi;th;psi;Z;u;v;w];
 Xdot = [xdot(1:3,1);xdot(9,1);xdot(10:12)];
 
 %% MEASUREMENT MODELS - ASSUMING THE FLOW SENSOR FRAME AND BODY FIXED FRAMES COINCIDE
 
 ax   = (v*r-w*q)-mu*u/m;
 ay   = (w*p-u*r)-mu*v/m;
 az   = (u*q-v*p)-T/m;
 xd   = u;
 yd   = v;
 h    = -Z;
 
 Y = [ax;ay;az;xd;yd;h];
 
 %% DERIVATIVES
 
 F1 = jacobian(Xdot,X);
 H1 = jacobian(Y,X);
 
 %%
 % ---------- EKF WITH MU ESTIMATED ----------------
 
  X    = [phi;th;psi;Z;u;v;w;mu];
 Xdot = [xdot(1:3,1);xdot(9,1);xdot(10:12);0];
 
 %% MEASUREMENT MODELS - ASSUMING THE FLOW SENSOR FRAME AND BODY FIXED FRAMES COINCIDE
 
 ax   = (v*r-w*q)-mu*u/m;
 ay   = (w*p-u*r)-mu*v/m;
 az   = (u*q-v*p)-T/m;
 xd   = u;
 yd   = v;
 h    = -Z;
 
 Y = [ax;ay;az;xd;yd;h];
 
 %% DERIVATIVES
 
 F2 = jacobian(Xdot,X);
 H2 = jacobian(Y,X);
 
 
 
 
 
