clear all;
clc;
clear functions;

syms x y z q1 q2 q3 q4 v1 v2 v3 w1 w2 w3
syms xf yf zf
syms Phi Theta Rho dT

%% System Jacobian

Omega = [   0     -w1  -w2  -w3
            w1     0   -w3   w2
            w2     w3    0  -w1
            w3    -w2   w1    0];        
I = eye(4);
F(1:3) = [x+v1*dT;y+v2*dT;z+v3*dT];
F(4:7) = (I+1/2*Omega*dT)*[q1;q2;q3;q4];
F(8:13) = [v1;v2;v3;w1;w2;w3];
dFdX = jacobian(F,[x y z q1 q2 q3 q4 v1 v2 v3 w1 w2 w3]);
matlabFunction(dFdX, 'file', 'dFdX.m');

Cbn = sym(zeros(3));

Cbn(1,1) = q1^2 + q2^2 - q3^2 - q4^2;
Cbn(1,2) = 2*(q2*q3 + q1*q4);
Cbn(1,3) = 2*(q2*q4 - q1*q3);
Cbn(2,1) = 2*(q2*q3 - q1*q4);
Cbn(2,2) = q1^2 - q2^2 + q3^2 - q4^2;
Cbn(2,3) = 2*(q3*q4 + q1*q2);
Cbn(3,1) = 2*(q2*q4 + q1*q3);
Cbn(3,2) = 2*(q3*q4 - q1*q2);
Cbn(3,3) = q1^2 - q2^2 - q3^2 + q4^2;

q1_inv = -q1;

Cbn_inv = sym(zeros(3));

Cbn_inv(1,1) = q1_inv^2 + q2^2 - q3^2 - q4^2;
Cbn_inv(1,2) = 2*(q2*q3 + q1_inv*q4);
Cbn_inv(1,3) = 2*(q2*q4 - q1_inv*q3);
Cbn_inv(2,1) = 2*(q2*q3 - q1_inv*q4);
Cbn_inv(2,2) = q1_inv^2 - q2^2 + q3^2 - q4^2;
Cbn_inv(2,3) = 2*(q3*q4 + q1_inv*q2);
Cbn_inv(3,1) = 2*(q2*q4 + q1_inv*q3);
Cbn_inv(3,2) = 2*(q3*q4 - q1_inv*q2);
Cbn_inv(3,3) = q1_inv^2 - q2^2 - q3^2 + q4^2;


%% Compass Augment
G = Cbn* [Rho*cos(Theta)*cos(Phi);Rho*cos(Theta)*sin(Phi);Rho*sin(Theta)] + [x;y;z];

dGdX = jacobian(G,[x y z q1 q2 q3 q4 v1 v2 v3 w1 w2 w3]);
dGdZ = jacobian(G,[xf yf zf]);

matlabFunction(dGdX, 'file', 'dGdX.m');
matlabFunction(dGdZ, 'file', 'dGdZ.m');


%% Compass Measurement Prediction
xd = xf-x;
yd = yf-y;
zd = zf-z;

fc = Cbn_inv*[xd;yd;zd];  %!!!!!!!!!!!!!!!!!!!!!!! ты остановился здесь

H = [atan(fc(2),fc(1));atan(fc(3),sqrt(fc(1)^2 +fc(2)^2)); sqrt(fc(1)^2 +fc(2)^2 +fc(3)^2)];

dHdX = jacobian(H,[x y z q1 q2 q3 q4 v1 v2 v3 w1 w2 w3]);

matlabFunction(dHdX, 'file', 'dHdX.m');