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

Gv = dFdX(dt,x(4),x(5),x(6),x(7),Wn(1),Wn(2),Wn(3));                      

Gu = dFdU(dt,x(4),x(5),x(6),x(7),Wn(1),Wn(2),Wn(3));


% % predict covariance
P(1:13,1:13)= Gv*P(1:13,1:13)*Gv' + Gu*Q*Gu';
if size(P,1)>13
    P(1:13,14:end)= Gv*P(1:13,14:end);
    P(14:end,1:13)= P(1:13,14:end)';
end    

% predict state

dcm = quat2dcm_cc(x(4:7)');
x(1:3) = x(1:3)+(dcm*[v,0,0]').*dt;

x(4:7)= mrotate_eml(x(4:7)', [Wn(1),Wn(2),Wn(3)],dt);
x(4:7) = quatnormalize(x(4:7)');

x(8:10) = dcm*[v,0,0]';

x(11:13) = [Wn(3),Wn(2),Wn(1)];


 

