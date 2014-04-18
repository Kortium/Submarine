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
xv(4:7) = mrotate_eml(xv(4:7), [W(1),W(2),W(3)],dt);
xv(4:7) = quatnormalize(xv(4:7));
xv(8:10) = DCM*[V,0,0]';
xv(11:13) = [W(3),W(2),W(1)];
