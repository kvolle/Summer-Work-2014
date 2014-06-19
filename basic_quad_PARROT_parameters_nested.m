  %function error = basic_quad()
%SUMMER PROJECT VERSION
clear all
clc

% Define simulation duration (tf = n/100)
n = 2000;

% Define copter properties
mass = 2.3969;%*(0.9 + rand()/5); %kg (with up to plus/minus 10% error)
mu = .05;%*(0.9 + rand()/5); %drag coefficient (up to plus/minus 10% error)

% Define Initial state
pos = [0;0;-5];
or = [rand()-0.5;rand()-0.5;(rand()-0.5)]/1000000;
vel = [0;0;0];
ang = [0;0;0];
State = [pos;or;vel;ang];

% Define Initial forces and moments
force = [0;0;0];
moment = [0;0;0];

% Define initial moment of inertia matrix
Ib = [0.002425 0 0;0 0.002577 0;0 0 .0059626];

inner_loop_gains = [15.50,0,-25,15.50,0,-25,15.0,0,-25,-.03,0,0];
% Instantiate object
copter = eulerRK4(State,Ib,mass,force,moment);
% Instantiate control system
cntrl = controlAR(copter.m,copter.Ib,mu,inner_loop_gains);
cntrl.geometry();

% Initialize simulated INS and Flow sensors
accelerometer = zeros(3,n);
gyro = zeros(3,n);
flow = zeros(2,n);
altimeter = zeros(n);

% Characterize the sensor noise/error
std_dev_acc = 1;
mean_err_acc = 0;
acc_drift_rate = [0;0;0];
std_dev_gyro = 1;
mean_err_gyro = 0;
gyro_drift_rate = [0;0;0];
std_dev_flow = 1;
mean_err_flow = 0;
flow_drift_rate = [0;0];
std_dev_altimeter = 1;
mean_err_altimeter = 0;
altimeter_drift_rate = 0;

% Initialize some debugging variables for efficiency
time = 0.01:0.01:n/100;
tmp = zeros(3,n);
thrust = zeros(4,n);
position = zeros(4,n);
vert_vel = zeros(n);
world_vel = zeros(4,n);
% Start timer on simulation, for evaluation purposes
tic
set_points = 0;
% Velocity has +/- 1m/s error in u,v and +/- 0.25 m/s error in w
cnst_err = zeros(12,1);%[0;0;0;0;0;0;2*rand()-1;2*rand()-1;.5*rand-.25;0;0;0];
path = zeros(4,n);
for i =1:n/2
    path(1,i) = 3;
    path(1,i+n/2) = 3;
end
fileIDstate = fopen('test_data_state.txt','a');
fileIDpath = fopen('test_data_path.txt','a');
fileIDinner = fopen('test_data_inner.txt','a');
fileIDthrust = fopen('test_data_thrust.txt','a');
for i = 1:n
    inner_loop_set_points = cntrl.outer_loop(copter.State+cnst_err,path(:,i));
    
    thrust(:,i) = cntrl.inner_loop(inner_loop_set_points,copter.State+cnst_err);
    
    fprintf(fileIDstate,'%f %f %f %f %f %f %f %f %f\n',...
        copter.State(4),copter.State(5),copter.State(6),copter.State(7),copter.State(8),copter.State(9),copter.State(10),copter.State(11),copter.State(12));
    fprintf(fileIDpath,'%f %f %f %f\n',path(1,i),path(2,i),path(3,i),path(4,i));
    fprintf(fileIDinner,'%f %f %f %f\n',inner_loop_set_points(1),inner_loop_set_points(2),inner_loop_set_points(3),inner_loop_set_points(4));
    fprintf(fileIDthrust,'%f %f %f %f\n',thrust(1,i),thrust(2,i),thrust(3,i),thrust(4,i));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %thrust(:,i) = [2.943 2.943 2.943 2.943]';
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Apply the commanded thrusts
    result = cntrl.A_actual*thrust(:,i);
    copter.Moment = result(1:3);
    copter.Force = [0;0;0];
    copter.Force(3) = result(4);
    
    % Correct for drag
    %%% Not 100% sure this is being done correctly
    copter.Force = copter.Force - (mu)*[copter.State(7:8);0];

    % Setting plotting/debugging variables
    tmp(:,i) = copter.State(4:6);
    position(1,i) = copter.State(1);
    position(2,i) = copter.State(2);
    position(3,i) = -copter.State(3);
    body_vel(:,i) = copter.State(7:9);
    vert_vel(i) = -copter.State(9);
    debug(:,i) = copter.Force;%(mu)*copter.State(7:9);
  
    % Update accelerometer reading
    accel_noise = std_dev_acc*rand(3,1) + mean_err_acc;
    accel_drift = acc_drift_rate*time(i);
    % Neglecting vertical drag in next line
    accelerometer(:,i) = -cross(copter.State(10:12),copter.State(7:9)) - [mu*copter.State(7:8)/mass;sum(thrust(:,i))/mass] + accel_noise + accel_drift;
    
    % Update gyroscope reading
    gyro_noise = std_dev_gyro*rand(3,1) + mean_err_gyro;
    gyro_drift = gyro_drift_rate*time(i);
    gyro(:,i) = copter.State(4:6)+gyro_noise + gyro_drift;
    
    % Update flow sensor reading
    flow_noise = std_dev_flow*rand(2,1) + mean_err_flow;
    flow_drift = flow_drift_rate*time(i);
    flow(:,i) = copter.State(7:8) + flow_noise + flow_drift;
    
    % Update altimeter reading
    altimeter_noise = std_dev_altimeter*rand(1,1) +mean_err_altimeter;
    altimeter_drift = altimeter_drift_rate*time(i);
    altimeter(i) = -copter.State(3) + altimeter_noise +altimeter_drift;
    
    % RK4 integrator
    copter.State = copter.homebrewRK4();
%copter.State(4:6) = inner_loop_set_points(1:3);
end
fclose('all');
toc
% Set output variable when used as a function for monte carlo simulation
error = sqrt(copter.State(4)^2 + copter.State(5)^2 + copter.State(6)^2);

plot_quad(time,tmp,position,vert_vel,thrust)
%{
plot(time,debug(1,:),'r');
hold on
plot(time,debug(2,:),'g');
plot(time,debug(3,:),'b');
%}
world_vel = zeros(3,n);
%
for i = 2:n
    %world_vel(:,i) = (position(:,i)-position(:,i-1))*100;
    world_vel(1,i) = (position(1,i)-position(1,i-1))*100;
    world_vel(2,i) = (position(2,i)-position(2,i-1))*100;
    world_vel(3,i) = (position(3,i)-position(3,i-1))*100;
end
figure(3)
plot(time,body_vel(1,:),'r');
hold on
plot(time,body_vel(2,:),'g');
plot(time,body_vel(3,:),'b');
title('Body Frame Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');

figure(4)
plot(time,world_vel(1,:),'r.-');
hold on
plot(time,world_vel(2,:),'g');
plot(time,world_vel(3,:),'b');
title('World Frame Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
figure(5)
plot(thrust(1,:),'r');
hold on
plot(thrust(2,:),'g');
plot(thrust(3,:),'b');
plot(thrust(4,:),'k');
%}