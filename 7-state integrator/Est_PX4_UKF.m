function [X,yhat,P,Pxy,Pvv] = Est_PX4_UKF(X,yhat,P,Pxy,Pvv,y,mu,t)

% ESTIMATOR TO ESTIMATE PHI, THETA, PSI, Z, U,V, W WHILE DRAG MU IS KNOWN

global Nv G R Q gyrog FT k1 k2 l Jx Jy Jz g m dt mu1

global F

nk  = length(X);
mk  = length(R);
qk  = length(Q);
L   = nk+mk+qk;
mu1 = mu;

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

%% CORRECTION STEP

Pxv = zeros(nk,mk);
Pwv = zeros(qk,mk);
Pxw = zeros(nk,qk);
nu  = y-yhat;
K   = Pxy*inv(Pvv);
P   = P - K*Pvv*K';
X   = X + K*nu;
Pa  = [P Pxw  Pxv; Pxw'  Q  Pwv; Pxv'  Pwv'  R];

%% GENERATING POINTS FOR COVARIANCE PROPOGATION

ka  = 3-L; % 1;%
alp = 0.001; % 10^-4 to 1
bet = 2;
lam = alp*(L+ka) - L;
ga  = sqrt(L+lam);
Sig = ga*[sqrtm(Pa), -sqrtm(Pa)];

%% PROPAGATE THE POINTS

Xa_hat  = [X;zeros(qk,1);zeros(mk,1)];
Xa(:,1) = Xa_hat;

for j = 1 : 2*L+1
    
    if j > 1
        Xa(:,j) = Xa_hat+Sig(:,j-1); 
    end    

    Xa_k1(:,j) = rk4_X_UKF(1,Xa(:,j));
    
end

%% WEIGHTS

Wmean(1) = lam/(L+lam);
Wcov(1)  = lam/(L+lam)+(1-alp^2+bet);
Wmean(2:2*L+1,1) = 1/(2*(L+lam));
Wcov(2:2*L+1,1)  = 1/(2*(L+lam));

%% PREDICTION

X   = Wmean(1)*Xa_k1(1:nk,1)+Wmean(2).*sum(Xa_k1(1:nk,2:end),2);

for j = 1 : 2*L+1
    
    phi = Xa_k1(1,j);  th  = Xa_k1(2,j); psi = Xa_k1(3,j); p   = gyrog(1); q   = gyrog(2); r   = gyrog(3); 
    Z   = Xa_k1(4,j);   u  = Xa_k1(5,j); v   = Xa_k1(6,j); w   = Xa_k1(7,j);
    h = [ r*v - q*w - (mu*u)/m; p*w - r*u - (mu*v)/m;  q*u - p*v - (FT(1))/m;  u;  v; -Z] + Xa_k1(15:20,j);
    Y(:,j) = h;
    
end

y = Wmean(1)*Y(:,1) + Wmean(2)*sum(Y(:,2:2*L+1),2);

Pxx = Wcov(1)*[Xa_k1(1:nk,1)-X]*[Xa_k1(1:nk,1)-X]';
Pyy = Wcov(1)*[Y(:,1)-y]*[Y(:,1)-y]';
Pxy = Wcov(1)*[Xa_k1(1:nk,1)-X]*[Y(:,1)-y]';
for j = 2 : 2*L+1
    Pxx = Pxx + Wcov(2)*[Xa_k1(1:nk,j)-X]*[Xa_k1(1:nk,j)-X]';
    Pyy = Pyy + Wcov(2)*[Y(:,j)-y]*[Y(:,j)-y]';
    Pxy = Pxy + Wcov(2)*[Xa_k1(1:nk,j)-X]*[Y(:,j)-y]';
end

yhat = y;

%% X- roll-pitch-yaw z(Inertial) u-v-w(Body)
 % Y- ax ay az ub vb h

Pvv = Pyy;
P   = Pxx;

