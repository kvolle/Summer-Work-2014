clear all
clc

% Implementation of the linear fixed gain observer
% As described in a paper by Leishman et. al.

% Using SI units
g = 9.81; % m/s^2
m = 2; % kg

mu = 0.25; % unitless

% x = [?,?,u,v]'
% u = [p,q,r]'
% y = [a_i,a_j]'

A = [0 0 0 0; 0 0 0 0;0 -g mu/m 0;g 0 0 mu/m];
B = [1 0 0;0 1 0;0 0 0;0 0 0];
C = [0 0 mu/m 0;0 0 0 mu/m];

% Find a Luenberger observer matrix
% Place poles in highly negative places
obs_pole_1 = -100;
obs_pole_2 = -105;
obs_pole_3 = -110;
obs_pole_4 = -115;
L = place(A',C',[obs_pole_1 obs_pole_2 obs_pole_3 obs_pole_4]);

K = [-1 0 0 0;0 -1.5 0 0;0 0 0 0];
At = [A-B*K B*K;zeros(size(A)) A-L*C]
Bt = [B;zeros(size(B))]
Ct = [C zeros(size(C))]
