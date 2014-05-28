function [X,P] = Est_PX4_UKF_mu(X,P,y,t)

% ESTIMATOR TO ESTIMATE PHI, THETA, PSI, Z, U,V, W WHILE DRAG MU IS KNOWN

global Nv G R gyrog FT k1 k2 l Jx Jy Jz g m R dt mu1

global F

%% ~~~~~~~~~~~~~~~~~~~EKF~~~~~~~~~~~~~~~~~~~

phi = X(1);
th  = X(2);
psi = X(3);
p   = gyrog(1);
q   = gyrog(2);
r   = gyrog(3);
Z   = X(4);
u   = X(5);
v   = X(6);
w   = X(7);
mu  = X(8);

%% X- roll-pitch-yaw z(Inertial) u-v-w(Body)
 % Y- ax ay az ub vb h

H = [[ 0, 0, 0,  0, -mu/m,     r, -q, -u/m]
[ 0, 0, 0,  0,    -r, -mu/m,  p, -v/m]
[ 0, 0, 0,  0,     q,    -p,  0,    0]
[ 0, 0, 0,  0,     1,     0,  0,    0]
[ 0, 0, 0,  0,     0,     1,  0,    0]
[ 0, 0, 0, -1,     0,     0,  0,    0]];

K = P*H'*inv(H*P*H' + R);

%% Update

h = [ r*v - q*w - (mu*u)/m
      p*w - r*u - (mu*v)/m
     q*u - p*v - (FT(1))/m
     u
     v
     -Z];

X = X + K*(y - h); 
P = (eye(length(X)) - K*H)*P;

%% Propogation

[tt,X]  = ode45(@eqn_X8,[t t+dt],X);
X  = X(end,:)';

F  = [[     q*cos(phi)*tan(th) - r*sin(phi)*tan(th),         r*cos(phi)*(tan(th)^2 + 1) + q*sin(phi)*(tan(th)^2 + 1), 0, 0,        0,                0,                0,    0]
[                   - r*cos(phi) - q*sin(phi),                                                               0, 0, 0,        0,                0,                0,    0]
[ (q*cos(phi))/cos(th) - (r*sin(phi))/cos(th), (r*cos(phi)*sin(th))/cos(th)^2 + (q*sin(phi)*sin(th))/cos(th)^2, 0, 0,        0,                0,                0,    0]
[     v*cos(phi)*cos(th) - w*cos(th)*sin(phi),           - u*cos(th) - w*cos(phi)*sin(th) - v*sin(phi)*sin(th), 0, 0, -sin(th), cos(th)*sin(phi), cos(phi)*cos(th),    0]
[                                           0,                                                      -g*cos(th), 0, 0,    -mu/m,                r,               -q, -u/m]
[                          g*cos(phi)*cos(th),                                             -g*sin(phi)*sin(th), 0, 0,       -r,            -mu/m,                p, -v/m]
[                         -g*cos(th)*sin(phi),                                             -g*cos(phi)*sin(th), 0, 0,        q,               -p,                0,    0]
[                                           0,                                                               0, 0, 0,        0,                0,                0,    0]];


P  = rk4_P(t,P);

