%function error = basic_quad()
%SUMMER PROJECT VERSION
clear all
clc

% Define 4 HTMs describing propeller location
thruster1 = [1 0 0 .0707;0 1 0 -.0707;0 0 1 0;0 0 0 1];
thruster2 = [1 0 0 -.0707;0 1 0 -.0707;0 0 1 0;0 0 0 1];
thruster3 = [1 0 0 -.0707;0 1 0 .0707;0 0 1 0;0 0 0 1];
thruster4 = [1 0 0 .0707;0 1 0 .0707;0 0 1 0;0 0 0 1];

% A matrix converts thrust commands to moments and total thrust
a = cross(thruster1(1:3,4),[0;0;-1]);
b = cross(thruster2(1:3,4),[0;0;-1]);
c = cross(thruster3(1:3,4),[0;0;-1]);
d = cross(thruster4(1:3,4),[0;0;-1]);
A = [a b c d];
A(4,:) = [-1 -1 -1 -1];
A = A + [0 0 0 0;0 0 0 0;.013 -.013 .013 -.013 ;0 0 0 0];

% Thrusters 1 and 3 rotate: CCW (posative yaw)
% Thrusters 2 and 4 rotate: CW (negative yaw)

mass = 3; %kg
mu = 0.1; %drag coefficient

% Define Initial state
pos = [0;0;-100];
or = [rand()-0.5;rand()-0.5;(rand()-0.5)];
vel = [0;0;0];
ang = [0;0;0];
State = [pos;or;vel;ang];

% Define Initial forces and moments
force = [0;0;0];
moment = [0;0;0];

% Define initial moment of inertia matrix
Ib = [0.0241 0 0;0 0.0232 0;0 0 .0451]*10;

% Instantiate object
copter = eulerRK4(State,Ib,mass,force,moment);

% Define commands to inner loop
% Eventually, these values will be set by outer loop
roll_desired = 0;
pitch_desired = 0;
yaw_desired = 0;

% PD attitude control
% P velocity control
% Steady-state error exists in velocity term, needs accounted for later
roll_p = 5;
roll_d = -3;
pitch_p = 5;
pitch_d = -3;
yaw_p = 5;
yaw_d = -3;
velocity_p = -30;

% Define simulation duration (tf = n/100)
n = 500;

% Initialize some debugging variables for efficiency
time = 0.01:0.01:n/100;
tmp = zeros(3,n);
thrust = zeros(4,n);
position = zeros(4,n);
vert_vel = zeros(n);

% Start timer on simulation, for evaluation purposes
tic
for i = 1:n
    % Calculate attitude error
    roll_err = roll_desired-copter.State(4);
    pitch_err = pitch_desired-copter.State(5);
    yaw_err = yaw_desired-copter.State(6);
    
    % Calculate rate of change of attitude error
    roll_accel = roll_p*roll_err + roll_d*copter.State(10);
    pitch_accel = pitch_p*pitch_err + pitch_d*copter.State(11);
    yaw_accel = yaw_p*yaw_err + yaw_d*copter.State(12);
    
    % Calculate desired moments
    moments = Ib*[roll_accel;pitch_accel;yaw_accel];
    
    % Convert moments to body frame
    % Doesn't end up mattering, so commented out
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
    
    % Calculate desired force along body Z
    force = mass*velocity_p*copter.State(9);

    % Calculate required thrust
    thrust(:,i) = A\[body_mom;force];

    
   % Apply thrust limiting (20 N / 4.5 lbf) 
    for j = 1:4
        if thrust(j,i)>20
            thrust(j,i) = 20;
        elseif thrust(j,i) < 0;
            thrust(j,i) = 0;
        end
    end
    
    % Apply the commanded thrusts
    result = A*thrust(:,i);
    copter.Moment = result(1:3);
    copter.Force(3) = result(4);
    % Correct for drag
    %%% Not 100% sure this is being done correctly
    copter.Force = copter.Force - (mu/mass)*copter.State(7:9);
    % Setting plotting/debugging variables
    tmp(:,i) = copter.State(4:6);
    position(1,i) = copter.State(1);
    position(2,i) = copter.State(2);
    position(3,i) = -copter.State(3);
    vert_vel(i) = -copter.State(9);
    
    % RK4 integrator
    copter.State = copter.homebrewRK4();
end
toc
% Set output variable when used as a function for monte carlo simulation
error = sqrt(copter.State(4)^2 + copter.State(5)^2 + copter.State(6)^2);

figure(1)
plot(time,tmp(1,:)*180/pi,'r');
hold on
plot(time,tmp(2,:)*180/pi,'g');
plot(time,tmp(3,:)*180/pi,'b');
title('Attitude Adjustment');
xlabel('Time (s)');
ylabel('Angle (Deg)');
legend('Roll','Pitch','Yaw');
%}
%
figure(2)
plot3(position(1,1:n/2),position(2,1:n/2),position(3,1:n/2));
hold on
plot3(position(1,n/2:n),position(2,n/2:n),position(3,n/2:n),'r');
axis equal
Title('Position');
xlabel('X Position');
ylabel('Y Position');
zlabel('Altitude');
legend('First half of simulation','Second half of simulation');
%}
%
figure(3)
plot(time,vert_vel,'k');
title('Velocity Along Body Z Axis');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
%}
%{
figure(4)
plot(time,thrust(1,:),'r');
hold on
plot(time,thrust(2,:),'g');
plot(time,thrust(3,:),'b');
plot(time,thrust(4,:),'k');
title('Thrust');
xlabel('Time (s)');
ylabel('Thrust (N)');
legend('Motor 1','Motor 2','Motor 3','Motor 4');
%}
