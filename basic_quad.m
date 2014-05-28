%function error = basic_quad()
%SUMMER PROJECT VERSION

%%%%%%%%%%%%%%%%%%%%%
% CONVERT TO METRIC %
%%%%%%%%%%%%%%%%%%%%%
thruster1 = [1 0 0 1;0 1 0 -1;0 0 1 0;0 0 0 1];
thruster2 = [1 0 0 -1;0 1 0 -1;0 0 1 0;0 0 0 1];
thruster3 = [1 0 0 -1;0 1 0 1;0 0 1 0;0 0 0 1];
thruster4 = [1 0 0 1;0 1 0 1;0 0 1 0;0 0 0 1];

a = cross(thruster1(1:3,4),[0;0;-1]);
b = cross(thruster2(1:3,4),[0;0;-1]);
c = cross(thruster3(1:3,4),[0;0;-1]);
d = cross(thruster4(1:3,4),[0;0;-1]);
A = [a b c d];
A(4,:) = [-1 -1 -1 -1];
A = A + [0 0 0 0;0 0 0 0;-1 1 -1 1 ;0 0 0 0];

% Thrusters 1 and 3 rotate: CW (negative yaw)
% Thrusters 2 and 4 rotate: CCw (positive yaw)

mass = 4/32.2;

pos = [0;0;-100];
or = [rand()-0.5;rand()-0.5;(rand()-0.5)];
vel = [0;0;0];
ang = [0;0;0];
State = [pos;or;vel;ang];

force = [0;0;0];
moment = [0;0;0];

Ib = [4 0 0;0 4 0;0 0 4*sqrt(2)];

copter = eulerRK4(State,Ib,mass,force,moment);

roll_desired = 0;
pitch_desired = 0;
yaw_desired = 0;
%{
roll_p = 8;
roll_d = -3;
pitch_p = 8;
pitch_d = -3;
yaw_p = 8;
yaw_d = -3;
%}
roll_p = 2.5;
roll_d = -1.5;
pitch_p = 2.5;
pitch_d = -1.5;
yaw_p = 2.5;
yaw_d = -1.5;
velocity_p = -0.2;

n = 1000;
tmp = zeros(3,n);
tmp2 = zeros(3,n);
thrust = zeros(4,n);
tic
for i = 1:n
    roll_err = roll_desired-copter.State(4);
    pitch_err = pitch_desired-copter.State(5);
    yaw_err = yaw_desired-copter.State(6);
    
    roll_accel = roll_p*roll_err + roll_d*copter.State(10);
    pitch_accel = pitch_p*pitch_err + pitch_d*copter.State(11);
    yaw_accel = yaw_p*yaw_err + yaw_d*copter.State(12);
    
    moments = Ib*[roll_accel;pitch_accel;yaw_accel];
    body_mom = moments;
    %{
    % Roll, Pitch, Yaw vector
    Or = copter.State(4:6);
    % Convert body coordinates to world coordinates
    body_to_world = [cos(Or(2))*cos(Or(3)) -cos(Or(2))*sin(Or(3)) sin(Or(2));...
                   cos(Or(1))*sin(Or(3))+cos(Or(3))*sin(Or(1))*sin(Or(2)) cos(Or(1))*cos(Or(3))-sin(Or(1))*sin(Or(2))*sin(Or(3)) -cos(Or(2))*sin(Or(1));...
                   sin(Or(1))*sin(Or(3))-cos(Or(1))*cos(Or(3))*sin(Or(2)) cos(Or(3))*sin(Or(1))+cos(Or(1))*sin(Or(2))*sin(Or(3)) cos(Or(1))*cos(Or(2))];
   
    % Switch the moments to body coordinates
    body_mom = body_to_world'*moments;
    %}
    accel = mass*velocity_p*copter.State(9);
%    copter.Moment = body_mom;
    %thrust(:,i) = [body_mom;accel]\A;
    thrust(:,i) = A\[body_mom;accel];
   %
    for j = 1:4
        if abs(thrust(j,i))>2.5
            thrust(j,i) = 2.5*thrust(j,i)/abs(thrust(j,i));
        end
    end
    %}
    result = A*thrust(:,i);
    copter.Moment = result(1:3);
    copter.Force(3) = result(4);
    tmp(:,i) = copter.State(4:6);
    tmp2(:,i) = copter.State(7:9);
    copter.State = copter.homebrewRK4();
end
toc
%error = sqrt(copter.State(4)^2 + copter.State(5)^2 + copter.State(6)^2);
%
hold on
plot(tmp(1,:)*180/pi,'r');
hold on
plot(tmp(2,:)*180/pi,'g');
plot(tmp(3,:)*180/pi,'b');
%}
%{
figure(2)
plot(thrust(1,:),'r');
hold on
plot(thrust(2,:),'g');
plot(thrust(3,:),'b');
plot(thrust(4,:),'k');
%}
%{
hold on
plot(tmp2(1,:),'r');
hold on
plot(tmp2(2,:),'g');
plot(tmp2(3,:),'b');
%}