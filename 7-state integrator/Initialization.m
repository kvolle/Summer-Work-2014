% --------INITIALIZATION-----------

X(:,1) = X0;
dF = 7*[0.1;0.1;0.15;0.1]; % Throttle setting of the four motors f-r-b-l

phi = X(1,1); th  = X(2,1); psi = X(3,1); p  = X(4,1); q  = X(5,1); r  = X(6,1);
xi  = X(7,1); yi  = X(8,1); zi  = X(9,1); u  = X(10,1); v = X(11,1); w = X(12,1);

% ACCELEROMETER READINGS
std_acc = 0.1;
mu_acc  = 0;
dr_acc  = 0;
n_acc = std_acc*rand(3,1) + mu_acc;
d_acc = dr_acc*t(1);
FT = [k1 k1 k1 k1;0 -l*k1 0  l*k1; l*k1 0  -l*k1 0; -k2  k2  -k2  k2]*dF;
Acc(:,1) =[(v*r-w*q) - mu*u/m; (w*p-u*r) - mu*v/m; (u*q-v*p)-FT(1)/m]+ n_acc + d_acc;

% GYRO READINGS
std_gyro = 0.01;
mu_gyro  = 0;
dr_gyro  = 0;
n_gyro = std_gyro*rand(3,1) + mu_gyro;
d_gyro = dr_gyro*t(1);
Gyro(:,1) = X(4:6,1) + n_gyro + d_gyro;

% FLOW SENSOR READINGS
std_flow = 0.1;
mu_flow  = 0;
dr_flow  = 0;
Ui = X(10:12,1);
ui = Ui(1); vi = Ui(2); wi = Ui(3);
n_flow = std_flow*rand(3,1) + mu_flow;
d_flow = dr_flow*t(1);
Flo(:,1) = [ui;vi;-X(9,1)] + n_flow + d_flow;

% ESTIMATOR INITIALIZATION
R         = diag([std_acc^2;std_acc^2;std_acc^2;std_flow^2;std_flow^2;std_flow^2]);
P         = diag([0.1;0.1;0.1;0.2;0.2;0.1;0.1]);
Nv        = 0.001;
G         = 1;
Q         = Nv*diag(ones(7,1));
Xhat(:,1) = [X(1:3,1);X(9,1);X(10:12,1)]+ sqrt(diag(P)).*rand(7,1);
Cov(:,1)  = sqrt(diag(P));

P_mu         = diag([0.1;0.1;0.1;0.5;0.5;0.5;0.5;0.1]);
Xhat_mu(:,1) = [X(1:3,1);X(9,1);X(10:12,1);mu]+ sqrt(diag(P_mu)).*rand(8,1);
Cov_mu(:,1)  = sqrt(diag(P_mu));

P_UKF         = diag([0.1;0.1;0.1;0.2;0.2;0.1;0.1]);
Pxy           = zeros(7,6); 
Pvv           = R;
Xhat_UKF(:,1) = [X(1:3,1);X(9,1);X(10:12,1)]+ sqrt(diag(P_UKF)).*rand(7,1);
Cov_UKF(:,1)  = sqrt(diag(P_UKF));
Z = Xhat_UKF(4,1); u = Xhat_UKF(5,1); v = Xhat_UKF(6,1); w = Xhat_UKF(7,1);
yhat          = [r*v - q*w - (mu*u)/m; p*w - r*u - (mu*v)/m; q*u - p*v - (FT(1))/m;u;v;-Z] + sqrt(diag(Pvv)).*rand(6,1) ; % This is estimation of the measurements

% P_UKF_mu         = diag([0.1;0.1;0.1;0.5;0.5;0.5;0.5;0.1]);
% Xhat_UKF_mu(:,1) = [X(1:3,1);X(9,1);X(10:12,1);mu]+ sqrt(diag(P_UKF_mu)).*rand(8,1);
% Cov_UKF_mu(:,1)  = sqrt(diag(P_UKF_mu));