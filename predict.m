function [x,P]= predict (x,P,v,Wn,Q,dt)
%[x,P]= predict (x,P, Vn,Wn,Q,dt);
%
% Inputs:
%   x, P - SLAM state and covariance
%   v, g - control inputs: velocity and gamma (steer angle)
%   Q - covariance matrix for velocity and gamma
%   WB - vehicle wheelbase
%   dt - timestep
%
% Outputs: 
%   xn, Pn - predicted state and covariance
%
% Tim Bailey 2004.

% jacobians   
Gv = eye(13);
Gv(1:3,8:10) = eye(3);

q1 = x(4);
q2 = x(5);
q3 = x(6);
q4 = x(7);

wx = 0;
wy = Wn(2);
wz = Wn(1);

Gv(4:7,11:13) = [ -(dt*q2)/2  -(dt*q3)/2  -(dt*q4)/2
                          (dt*q1)/2  -(dt*q4)/2 (dt*q3)/2
                          (dt*q4)/2 (dt*q1)/2  -(dt*q2)/2
                           -(dt*q3)/2 (dt*q2)/2 (dt*q1)/2];
                       
Gv(4:7,4:7) = [ 1 -(dt*wx)/2  -(dt*wz)/2  -(dt*wy)/2
                      (dt*wx)/2 1 (dt*wz)/2 -(dt*wy)/2
                      (dt*wy)/2 -(dt*wz)/2 1 (dt*wx)/2
                      (dt*wz)/2 (dt*wy)/2 -(dt*wx)/2 1];
                         
% % predict covariance
P(1:13,1:13)= Gv*P(1:13,1:13)*Gv';
if size(P,1)>13
    P(1:13,14:end)= Gv*P(1:13,14:end);
    P(14:end,1:13)= P(1:13,14:end)';
end    

% predict state

dcm = quat2dcm(x(4:7)');
x(1:3) = x(1:3)+([v,0,0]*dcm)'.*dt;

x(4:7)= mrotate_eml(x(4:7)', [0,Wn(2),Wn(1)],dt);

x(8:10) = [v,0,0]*dcm;

x(11:13) = [0,Wn(2),Wn(1)];


 

