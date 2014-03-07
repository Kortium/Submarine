function xv= vehicle_model(xv, V,W,dt)
%
% INPUTS:
%   xv - vehicle pose [x;y;z;q]
%   V  - velocity
%   W - angular velocity vector [wz; wy; wx]
%   dt - change in time
%
% OUTPUTS:
%   xv - new vehicle pose

DCM = quat2dcm_cc(xv(4:7));
dp = (DCM*[V;0;0])'.*dt;


xv(1:3) = xv(1:3) + dp;

w1=W(1);
w2=W(2);
w3=W(3);
Omega = [   0     -w1  -w2  -w3
            w1     0   -w3   w2
            w2     w3    0  -w1
            w3    -w2   w1    0];  
I = eye(4); 
q1 = xv(4);
q2 = xv(5);
q3 = xv(6);
q4 = xv(7);

xv(4:7) = (I+1/2*Omega*dt)*[q1;q2;q3;q4];   
xv(4:7) = quatnormalize(xv(4:7));
xv(8:10) = DCM*[V,0,0]';
xv(11:13) = [W(3),W(2),W(1)];
