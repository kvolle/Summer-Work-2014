function xdot = eqn_X(t,x)

global FT k1 k2 l Jx Jy Jz g mu m

phi = x(1);
th  = x(2);
psi = x(3);
p   = x(4);
q   = x(5);
r   = x(6);

X   = x(7);
Y   = x(8);
Z   = x(9);
u   = x(10);
v   = x(11);
w   = x(12);

 %% States - roll-pitch-yaw p-q-r x-y-z(Inertial) u-v-w(Body)
 
 Rt = [1 sin(phi)*tan(th)  cos(phi)*tan(th); 0  cos(phi)  -sin(phi); 0  sin(phi)/cos(th)  cos(phi)/cos(th)];
 xdot(1:3,1) = Rt*x(4:6,1);
 
 F       = FT(1);
 tau_phi = FT(2);
 tau_th  = FT(3);
 tau_psi = FT(4);
 xdot(4:6,1) = [((Jy-Jz)*q*r + tau_phi)/Jx; ((Jz-Jx)*p*r + tau_th)/Jy; ((Jx-Jy)*p*q + tau_psi)/Jz];
 
 xdot(7:9,1)   = (Ex(phi)*Ey(th)*Ez(psi))'*x(10:12,1);
 
 T  = F; % Thrust at CG along body z-axis
 xdot(10:12,1) = [-g*sin(th)+(v*r-w*q) - mu*u/m; g*sin(phi)*cos(th)+(w*p-u*r)-mu*v/m; g*cos(phi)*cos(th)+(u*q-v*p)-T/m];
 
