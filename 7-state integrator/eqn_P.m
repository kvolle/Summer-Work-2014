function Pdot = eqn_P(t,P)

global F Nv G

% P is the covariance matrix, U is the covariance of noise
l  = length(P);
G1 = G*ones(1,l);
Nv1= Nv*diag(ones(l,1));
Pdot = F*P + P*F' + G1*Nv1*G1';



