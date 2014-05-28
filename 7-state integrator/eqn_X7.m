function xdot = eqn_X7(t,x)

global FT k1 k2 l Jx Jy Jz g mu1 m gyrog

mu  = mu1;
p   = gyrog(1);
q   = gyrog(2);
r   = gyrog(3);
phi = x(1);
th  = x(2);
psi = x(3);
Z   = x(4);
u   = x(5);
v   = x(6);
w   = x(7);

 %% States - roll-pitch-yaw p-q-r x-y-z(Inertial) u-v-w(Body)
 
 Rt = [1 sin(phi)*tan(th)  cos(phi)*tan(th); 0  cos(phi)  -sin(phi); 0  sin(phi)/cos(th)  cos(phi)/cos(th)];
 xdot(1:3,1) = Rt*[p;q;r];
 
 F  = FT(1);  
 
 xx   = (Ex(phi)*Ey(th)*Ez(psi))'*x(5:7,1); 
 xdot(4,1) = xx(3);
 T  = F; % Thrust at CG along body z-axis
 xdot(5:7,1) = [-g*sin(th)+(v*r-w*q) - mu*u/m; g*sin(phi)*cos(th)+(w*p-u*r)-mu*v/m; g*cos(phi)*cos(th)+(u*q-v*p)-T/m];
 
