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

DCM = quat2dcm(xv(4:7));
xv(1:3) = xv(1:3)+([V,0,0]*DCM).*dt;
xv(4:7) = mrotate_eml(xv(4:7), [W(3),W(2),W(1)],dt);
xv(8:10) = [V,0,0]*DCM;
xv(11:13) = [W(3),W(2),W(1)];
