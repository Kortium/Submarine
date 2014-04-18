function dcm = quat2dcm_syms( q )
%  QUAT2DCM Convert quaternion to direction cosine matrix.

qin = quatnormalize_eml( q );

% dcm = zeros(3,3,'single');
syms dcm

dcm(1,1) = qin(1)^2 + qin(2)^2 - qin(3)^2 - qin(4)^2;
dcm(1,2) = 2*(qin(2)*qin(3) + qin(1)*qin(4));
dcm(1,3) = 2*(qin(2)*qin(4) - qin(1)*qin(3));
dcm(2,1) = 2*(qin(2)*qin(3) - qin(1)*qin(4));
dcm(2,2) = qin(1)^2 - qin(2)^2 + qin(3)^2 - qin(4)^2;
dcm(2,3) = 2*(qin(3)*qin(4) + qin(1)*qin(2));
dcm(3,1) = 2*(qin(2)*qin(4) + qin(1)*qin(3));
dcm(3,2) = 2*(qin(3)*qin(4) - qin(1)*qin(2));
dcm(3,3) = qin(1)^2 - qin(2)^2 - qin(3)^2 + qin(4)^2;
