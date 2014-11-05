function [r1 r2 r3] = quat2angle_syms( q )%#eml
%   quat2angle_eml Convert quaternion to rotation angles
%   Embedded Matlab ready

%Normalize quaternion 
qin = q./(sqrt(sum(q.^2,2))* ones(1,4));

%DCM elements
c21 = 2*(qin(2)*qin(3) + qin(1)*qin(4));
c11 = qin(1)^2 + qin(2)^2 - qin(3)^2 - qin(4)^2;
c31 = 2*(qin(2)*qin(4) - qin(1)*qin(3));
c32 = 2*(qin(3)*qin(4) + qin(1)*qin(2));
c33 = qin(1)^2 - qin(2)^2 - qin(3)^2 + qin(4)^2;

%Angles from DCM
r1 = atan2(c21,c11);
r2 = asin(-c31);
r3 = atan2(c32,c33);

end