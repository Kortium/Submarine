clear all;
clc;
clear functions;

syms x y z q1 q2 q3 q4 v1 v2 v3 w1 w2 w3
syms wu1 wu2 wu3 v
syms xf yf zf
syms Phi Theta Rho dT

%% System Jacobian

q = [q1 q2 q3 q4];

Omega = [   0     -w1  -w2  -w3
            w1     0   -w3   w2
            w2     w3    0  -w1
            w3    -w2   w1    0];        
I = eye(4);
F(1:3) = [x+v1*dT;y+v2*dT;z+v3*dT];
F(4:7) = mrotate_eml([q1,q2,q3,q4], [w1,w2,w3],dT);
F(4:7) = quatnormalize_eml([F(4),F(5),F(6),F(7)]);
F(8:13) = [v1;v2;v3;w1;w2;w3];
dFdX = jacobian(F,[x y z q1 q2 q3 q4 v1 v2 v3 w1 w2 w3]);
matlabFunction(dFdX, 'file', 'dFdX.m');

DCM = quat2dcm_cc(F(4:7));
dp = (DCM*[v;0;0])'.*dT;

F(1:3) = F(1:3)+dp;

Fx = w1*dT;
Fy = w2*dT;
Fz = w3*dT;
Fm = sqrt(Fx*Fx+Fy*Fy+Fz*Fz);
sinFm2 = sin(Fm/2);
cosFm2 = cos(Fm/2); 

Nd = [cosFm2, Fx/Fm*sinFm2, Fy/Fm*sinFm2, Fz/Fm*sinFm2];

F(4:7) = [Nd(1)*q(1)-Nd(2:4)*q(2:4)' Nd(1)*q(2:4)+q(1)*Nd(2:4)+cross(q(2:4),Nd(2:4))];
F(8:10) = F(8:10) + (DCM*[v;0;0])';
F(11:13) = F(11:13) + [wu1;wu2;wu3]';

dFdU = jacobian(F,[wu1 wu2 wu3 v]);
matlabFunction(dFdU, 'file', 'dFdU.m');

Cbn = sym(zeros(3));

Cbn(1,1) = 1 - 2*(q3^2 + q4^2);
Cbn(1,2) = 2*(q2*q3 - q4*q1);
Cbn(1,3) = 2*(q2*q4 + q3*q1);
Cbn(2,1) = 2*(q2*q3 + q4*q1);
Cbn(2,2) = 1 - 2*(q2^2 + q4^2);
Cbn(2,3) = 2*(q3*q4 - q2*q1);
Cbn(3,1) = 2*(q2*q4 - q3*q1);
Cbn(3,2) = 2*(q3*q4 + q2*q1);
Cbn(3,3) = 1 - 2*(q2^2 + q3^2);

Cbn_inv = sym(zeros(3));
q1_inv = -q1;

Cbn_inv(1,1) = 1 - 2*(q3^2 + q4^2);
Cbn_inv(1,2) = 2*(q2*q3 - q4*q1_inv);
Cbn_inv(1,3) = 2*(q2*q4 + q3*q1_inv);
Cbn_inv(2,1) = 2*(q2*q3 + q4*q1_inv);
Cbn_inv(2,2) = 1 - 2*(q2^2 + q4^2);
Cbn_inv(2,3) = 2*(q3*q4 - q2*q1_inv);
Cbn_inv(3,1) = 2*(q2*q4 - q3*q1_inv);
Cbn_inv(3,2) = 2*(q3*q4 + q2*q1_inv);
Cbn_inv(3,3) = 1 - 2*(q2^2 + q3^2);

%% Compass Augment
G = Cbn* [Rho*cos(Theta)*cos(Phi);Rho*cos(Theta)*sin(Phi);Rho*sin(Theta)] + [x;y;z];

dGdX = jacobian(G,[x y z q1 q2 q3 q4 v1 v2 v3 w1 w2 w3]);
dGdZ = jacobian(G,[Phi Theta Rho]);

matlabFunction(dGdX, 'file', 'dGdX.m');
matlabFunction(dGdZ, 'file', 'dGdZ.m');


%% Compass Measurement Prediction
xd = x-xf;
yd = y-yf;
zd = z-zf;

fc = Cbn*[xd;yd;zd];  

H = [atan(fc(2),fc(1));atan(fc(3),sqrt(fc(1)^2 +fc(2)^2)); sqrt(fc(1)^2 +fc(2)^2 +fc(3)^2)];

dHdX = jacobian(H,[x y z q1 q2 q3 q4 v1 v2 v3 w1 w2 w3]);

matlabFunction(dHdX, 'file', 'dHdX.m');

dHdf = jacobian(H,[xf yf zf]);

matlabFunction(dHdf, 'file', 'dHdf.m');