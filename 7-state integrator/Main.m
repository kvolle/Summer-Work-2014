%%  QUADROTOR ESTIMATION AND CONTROL

clc
clear all
close all

global dt FT k1 k2 l Jx Jy Jz g mu m gyrog R G Nv Q

%% CONSTANTS

mu = 0.1; % Drag Coefficient
Jx = 0.0241;
Jy = 0.0232;
Jz = 0.0451;
k1 = 1; % Throttle to thrust constant
k2 = 1; % Constant for Torque on each motor as a funciton of throttle setting
l  = 0.10; % meter distance from CG to motor - assuming 20cm long center rod
m  = 0.3; % Mass approximate
g  = 9.8;

X0 = [0.1;0.1;0.1;0;0;0.1;0;0;-10;0.1;0;0]; % States - roll-pitch-yaw p-q-r x-y-z(Inertial) u-v-w(Body)
dt = 0.01;
tf = 10;
t  = 0 : dt : tf;

%% DYNAMICS SIMULATION - roll-pitch-yaw p-q-r x-y-z(Inertial) u-v-w(Body)

Initialization

% CONTROL GAINS AND FILTER PARAMETERS
Kp_phi = 0; Ki_phi = 0; Kd_phi = 0;
Kp_th  = 0; Ki_th  = 0; Kd_th  = 0;
Kp_psi = 0; Ki_psi = 0; Kd_psi = 0;
Kp_T   = 0; Ki_T   = 0; Kd_T   = 0;

%---------INTEGRATION--------------

for i = 1 : length(t)-1    

    X(:,i+1) = rk4_X(t(i),X(:,i)); 
    X(1:3,i+1) = pi2pi(X(1:3,i+1));
    
    % --------ESTIMATION ROUTINES
    gyrog = Gyro(:,i);
    
    % ESTIMATOR WITH DRAG TERM KNOWN 
    [Xhat(:,i+1),P] = Est_PX4(Xhat(:,i),P,[Acc(:,i);Flo(:,i)],mu,t(i));
    Xhat(1:3,i+1) = pi2pi(Xhat(1:3,i+1));
    Error(:,i+1)  = [X(1:3,i+1);X(9,i+1);X(10:12,i+1)] - Xhat(:,i+1);
    Cov(:,i+1)    = sqrt(diag(P));
    
    % ESTIMATING THE DRAG COEFFICIENT NOT KNOWN
    [Xhat_mu(:,i+1),P_mu] = Est_PX4_mu(Xhat_mu(:,i),P_mu,[Acc(:,i);Flo(:,i)],t(i));
    Xhat_mu(1:3,i+1) = pi2pi(Xhat_mu(1:3,i+1));
    Error_mu(:,i+1)  = [X(1:3,i+1);X(9,i+1);X(10:12,i+1);mu] - Xhat_mu(:,i+1);
    Cov_mu(:,i+1)    = sqrt(diag(P_mu));        
    
    % ESTIMATOR WITH DRAG TERM KNOWN 
    [Xhat_UKF(:,i+1),yhat,P_UKF,Pxy,Pvv] = Est_PX4_UKF(Xhat_UKF(:,i),yhat,P_UKF,Pxy,Pvv,[Acc(:,i);Flo(:,i)],mu,t(i));
    Xhat_UKF(1:3,i+1) = pi2pi(Xhat_UKF(1:3,i+1));
    Error_UKF(:,i+1)  = [X(1:3,i+1);X(9,i+1);X(10:12,i+1)] - Xhat_UKF(:,i+1);
    Cov_UKF(:,i+1)    = sqrt(diag(P_UKF));
    
    % ESTIMATING THE DRAG COEFFICIENT NOT KNOWN
%     [Xhat_UKF_mu(:,i+1),P_UKF_mu] = Est_PX4_UKF_mu(Xhat_UKF_mu(:,i),P_UKF_mu,[Acc(:,i);Flo(:,i)],t(i));
%     Xhat_UKF_mu(1:3,i+1) = pi2pi(Xhat_UKF_mu(1:3,i+1));
%     Error_UKF_mu(:,i+1)  = [X(1:3,i+1);X(9,i+1);X(10:12,i+1);mu] - Xhat_UKF_mu(:,i+1);
%     Cov_UKF_mu(:,i+1)    = sqrt(diag(P_UKF_mu));    
    
    % ------------CONTROL DESIGN-----------------
    if i < 50
        dF = 8*[0.1;0.1;0.1;0.1]; % Throttle setting of the four motors f-r-b-l
        FT = [k1 k1 k1 k1;0 -l*k1 0  l*k1; l*k1 0  -l*k1 0; -k2  k2  -k2  k2]*dF;
    elseif i > 50 && i < 60
        dF = 7*[0.1;0.1;0.15;0.1]; % Throttle setting of the four motors f-r-b-l
        FT = [k1 k1 k1 k1;0 -l*k1 0  l*k1; l*k1 0  -l*k1 0; -k2  k2  -k2  k2]*dF;
    else
        dF = 8*[0.1;0.1;0.1;0.1]; % Throttle setting of the four motors f-r-b-l
        FT = [k1 k1 k1 k1;0 -l*k1 0  l*k1; l*k1 0  -l*k1 0; -k2  k2  -k2  k2]*dF;
    end       
    
    % ERRORS    
%     FT(1) = Kp_T*e_h + Ki_T*e_hi + Kd_T*e_hd;
%     FT(2) = Kp_phi*e_h + Ki_phi*e_hi + Kd_phi*e_hd;
%     FT(3) = Kp_th*e_h + Ki_th*e_hi + Kd_th*e_hd;
%     FT(4) = Kp_psi*e_h + Ki_psi*e_hi + Kd_psi*e_hd;
    
        
    % -------NEXT MEASUREMENTS
    phi = X(1,i+1); th  = X(2,i+1); psi = X(3,i+1); p  = X(4,i+1); q  = X(5,i+1); r  = X(6,i+1);
    xi  = X(7,i+1); yi  = X(8,i+1); zi  = X(9,i+1); u  = X(10,i+1); v = X(11,i+1); w = X(12,i+1);
    
    % ACCELEROMETER READINGS 
    n_acc = std_acc*rand(3,1) + mu_acc;
    d_acc = dr_acc*t(i);
    FT = [k1 k1 k1 k1;0 -l*k1 0  l*k1; l*k1 0  -l*k1 0; -k2  k2  -k2  k2]*dF;
    Acc(:,i+1) = [(v*r-w*q) - mu*u/m; (w*p-u*r) - mu*v/m; (u*q-v*p)-FT(1)/m]+ n_acc + d_acc;
    
    % GYRO READINGS
    n_gyro = std_gyro*rand(3,1) + mu_gyro;
    d_gyro = dr_gyro*t(i);
    Gyro(:,i+1) = X(4:6,i+1) + n_gyro + d_gyro;
        
    % MAGNETOMETER READINGS
    % -- code later
    
   % FLOW SENSOR READINGS  
    Ui = X(10:12,i+1);
    ui = Ui(1); vi = Ui(2); wi = Ui(3);  
    n_flow = std_flow*rand(3,1) + mu_flow;
    d_flow = dr_flow*t(i);
    Flo(:,i+1) = [ui;vi;-X(9,i+1)]+ n_flow + d_flow; % x-y camera reference frame velocity and altitude(ground distance-assuming flat) in inertial frame
   
   
end

%% ESTIMATION

%% CONTROL

%% PLOT

close all

figure
subplot(2,3,1)
plot(t,X(1,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf \phi');grid on
subplot(2,3,2)
plot(t,X(2,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf \theta');grid on;title('\bf Rotational States');
subplot(2,3,3)
plot(t,X(3,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf \psi');grid on
subplot(2,3,4)
plot(t,X(4,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf p');grid on
subplot(2,3,5)
plot(t,X(5,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf q');grid on
subplot(2,3,6)
plot(t,X(6,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf r');grid on



figure
subplot(2,3,1)
plot(t,X(7,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf X');grid on
subplot(2,3,2)
plot(t,X(8,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf Y');grid on; title('\bf Translational States');
subplot(2,3,3)
plot(t,X(9,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf Z (Positive-Down)');grid on
subplot(2,3,4)
plot(t,X(10,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf u');grid on
subplot(2,3,5)
plot(t,X(11,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf v');grid on
subplot(2,3,6)
plot(t,X(12,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf w');grid on

figure
subplot(2,3,1)
plot(t,Acc(1,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf a_x');grid on
subplot(2,3,2)
plot(t,Acc(2,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf a_y');grid on; title('\bf Accelerometer and Gyro Measurements');
subplot(2,3,3)
plot(t,Acc(3,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf a_z');grid on
subplot(2,3,4)
plot(t,Gyro(1,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf p');grid on
subplot(2,3,5)
plot(t,Gyro(2,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf q');grid on
subplot(2,3,6)
plot(t,Gyro(3,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf r');grid on

figure
subplot(3,1,1)
plot(t,Flo(1,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf u');grid on; title('Flow Sensor Output');
subplot(3,1,2)
plot(t,Flo(2,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf v');grid on
subplot(3,1,3)
plot(t,Flo(3,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf h');grid on

figure
subplot(2,3,1)
plot(t,X(1,:),'k',t,Xhat(1,:),'-r',t,Xhat_UKF(1,:),'-b','LineWidth',2);xlabel('\bf t');ylabel('\bf \phi');grid on
subplot(2,3,2)
plot(t,X(1,:),'k',t,Xhat(2,:),'-r',t,Xhat_UKF(2,:),'-b','LineWidth',2);ylabel('\bf \theta');grid on; title('\bf Estimated versus Actual States - Known Drag Coefficient');
subplot(2,3,3)
plot(t,X(3,:),'k',t,Xhat(3,:),'-r',t,Xhat_UKF(3,:),'-b','LineWidth',2);xlabel('\bf t');ylabel('\bf \psi');grid on
subplot(2,3,4)
plot(t,X(9,:),'k',t,Xhat(4,:),'-r',t,Xhat_UKF(4,:),'-b',t,-Flo(3,:),'-m','LineWidth',2);xlabel('\bf t');ylabel('\bf Z');grid on; legend('Actual', 'Estimated', 'Measured');
subplot(2,3,5)
plot(t,X(10,:),'k',t,Xhat(5,:),'-r',t,Xhat_UKF(5,:),'-b',t,Flo(1,:),'-m','LineWidth',2);xlabel('\bf t');ylabel('\bf u');grid on
subplot(2,3,6)
plot(t,X(11,:),'k',t,Xhat(6,:),'-r',t,Xhat_UKF(6,:),'-b',t,Flo(2,:),'-m','LineWidth',2);xlabel('\bf t');ylabel('\bf v');grid on

figure
subplot(2,3,1)
plot(t,Error(1,:),'r',t,Error_UKF(1,:),'b',t,Cov(1,:),'k',t,-Cov(1,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf Error in \phi');grid on
subplot(2,3,2)
plot(t,Error(2,:),'r',t,Error_UKF(2,:),'b',t,Cov(2,:),'k',t,-Cov(2,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf Error in \theta');grid on; title('\bf Estimated versus Actual States - Known Drag Coefficient');
subplot(2,3,3)
plot(t,Error(3,:),'r',t,Error_UKF(3,:),'b',t,Cov(3,:),'k',t,-Cov(3,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf Error in \psi');grid on
subplot(2,3,4)
plot(t,Error(4,:),'r',t,Error_UKF(4,:),'b',t,Cov(4,:),'k',t,-Cov(4,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf Error in h');grid on
subplot(2,3,5)
plot(t,Error(5,:),'r',t,Error_UKF(5,:),'b',t,Cov(5,:),'k',t,-Cov(5,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf Error in u');grid on
subplot(2,3,6)
plot(t,Error(6,:),'r',t,Error_UKF(6,:),'b',t,Cov(6,:),'k',t,-Cov(6,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf Error in v');grid on


figure
subplot(2,3,1)
plot(t,X(1,:),'k',t,Xhat_mu(1,:),'.-b','LineWidth',2);xlabel('\bf t');ylabel('\bf \phi');grid on
subplot(2,3,2)
plot(t,X(2,:),'k',t,Xhat_mu(2,:),'.-b','LineWidth',2);xlabel('\bf t');ylabel('\bf \theta');grid on; title('\bf Estimated versus Actual States - Estimated Drag Coefficient');
subplot(2,3,3)
plot(t,mu,'k',t,Xhat_mu(end,:),'.-b','LineWidth',2);xlabel('\bf t');ylabel('\bf \mu');grid on
subplot(2,3,4)
plot(t,X(9,:),'k',t,Xhat_mu(4,:),'.-b',t,-Flo(3,:),'.-r','LineWidth',2);xlabel('\bf t');ylabel('\bf Z');grid on; legend('Actual', 'Estimated', 'Measured');
subplot(2,3,5)
plot(t,X(10,:),'k',t,Xhat_mu(5,:),'.-b',t,Flo(1,:),'.-r','LineWidth',2);xlabel('\bf t');ylabel('\bf u');grid on
subplot(2,3,6)
plot(t,X(11,:),'k',t,Xhat_mu(6,:),'.-b',t,Flo(2,:),'.-r','LineWidth',2);xlabel('\bf t');ylabel('\bf v');grid on

figure
subplot(2,3,1)
plot(t,Error_mu(1,:),'r',t,Cov_mu(1,:),'k',t,-Cov_mu(1,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf Error in \phi');grid on
subplot(2,3,2)
plot(t,Error_mu(2,:),'r',t,Cov_mu(2,:),'k',t,-Cov_mu(2,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf Error in \theta');grid on; title('\bf Estimated versus Actual States - Estimated Drag Coefficient');
subplot(2,3,3)
plot(t,Error_mu(end,:),'r',t,Cov_mu(end,:),'k',t,-Cov_mu(end,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf Error in \mu');grid on
subplot(2,3,4)
plot(t,Error_mu(4,:),'r',t,Cov_mu(4,:),'k',t,-Cov_mu(4,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf Error in h');grid on
subplot(2,3,5)
plot(t,Error_mu(5,:),'r',t,Cov_mu(5,:),'k',t,-Cov_mu(5,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf Error in u');grid on
subplot(2,3,6)
plot(t,Error_mu(6,:),'r',t,Cov_mu(6,:),'k',t,-Cov_mu(6,:),'k','LineWidth',2);xlabel('\bf t');ylabel('\bf Error in v');grid on

