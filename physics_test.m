clear all
clc


mass = 1;
% Define Initial state
pos = [0;0;0];
%or = [rand()-0.5;rand()-0.5;(rand()-0.5)]/1000000;
Or = [0;0;-pi/4];
vel = [1;0;0];
ang = [0;0;0];
State = [pos;Or;vel;ang];
Rd = [cos(Or(2))*cos(-Or(3)), sin(Or(1))*sin(Or(2))*cos(-Or(3))-cos(Or(1))*sin(-Or(3)), cos(Or(1))*sin(Or(2))*cos(-Or(3))-sin(Or(1))*sin(-Or(3));
                  -cos(Or(2))*sin(-Or(3)), -sin(Or(1))*sin(Or(2))*sin(-Or(3))+cos(Or(1))*cos(-Or(3)), -cos(Or(1))*sin(Or(2))*sin(-Or(3))+sin(Or(1))*cos(-Or(3));
                -sin(Or(2)), sin(Or(1))*cos(Or(2)), cos(Or(1))*cos(Or(2))]
% Define Initial forces and moments
force = [0;0;0];
moment = [0;0;0];

% Define initial moment of inertia matrix
Ib = [0.0241 0 0;0 0.0232 0;0 0 .0451]*10;

copter = eulerRK4(State,Ib,mass,force,moment);
n = 1000;
for i = 1:n
   t(i) = i/100; 
   body_vel(:,i) = copter.State(7:9);
   wv(:,i) = Rd*copter.State(7:9);
   position(:,i) = copter.State(1:3);
   copter.State = copter.homebrewRK4();
end
world_vel = zeros(3,n);
for i =2:n
    world_vel(1,i) = (position(1,i)-position(1,i-1))*100;
    world_vel(2,i) = (position(2,i)-position(2,i-1))*100;
    world_vel(3,i) = -(position(3,i)-position(3,i-1))*100;
end
plot(t,world_vel(1,:),'r');
hold on
plot(t,world_vel(2,:),'g');

%plot(t,wv(1,:),'r.');
%plot(t,wv(2,:),'g.');