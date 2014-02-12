function [x,P]= augment(x,P,z,R)
%function [x,P]= augment(x,P,z,R)
%
% Inputs:
%   x, P - SLAM state and covariance
%   z, R - range-bearing measurements and covariances, each of a new feature
%
% Outputs:
%   x, P - augmented state and covariance
%
% Notes: 
%   - We assume the number of vehicle pose states is three.
%   - Only one value for R is used, as all measurements are assume d to 
%   have same noise properties.
%
% Tim Bailey 2004.

% add new features to state
for i=1:size(z,2)
    [x,P]= add_one_z(x,P,z(:,i),R);
end

%
%

function [x,P]= add_one_z(x,P,z,R)

len= length(x);
phi= z(1); teta = z(2); rho= z(3);

q1 = x(4);
q2 = x(5);
q3 = x(6);
q4 = x(7);
% 
DCM = quat2dcm(x(4:7)');
% augment x

cfc = DCM * [rho*cos(teta)*cos(phi);  rho*cos(teta)*sin(phi);  rho*sin(teta)];

x= [x;
    x(1) - cfc(1);
    x(2) - cfc(2);
    x(3) - cfc(3)];

% jacobians

% Gv = [ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
%            0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
%            0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];


Gv = [ 1, 0, 0,                           (2*rho*(- sin(teta)*q1^2*q3 + cos(teta)*sin(phi)*q1^2*q4 + 2*cos(teta)*sin(phi)*q1*q2*q3 + 2*sin(teta)*q1*q2*q4 - 2*cos(phi)*cos(teta)*q1*q3^2 - 2*cos(phi)*cos(teta)*q1*q4^2 + sin(teta)*q2^2*q3 - cos(teta)*sin(phi)*q2^2*q4 + sin(teta)*q3^3 - cos(teta)*sin(phi)*q3^2*q4 + sin(teta)*q3*q4^2 - cos(teta)*sin(phi)*q4^3))/(q1^2 + q2^2 + q3^2 + q4^2)^2,                           -(2*rho*(cos(teta)*sin(phi)*q1^2*q3 + sin(teta)*q1^2*q4 + 2*sin(teta)*q1*q2*q3 - 2*cos(teta)*sin(phi)*q1*q2*q4 - cos(teta)*sin(phi)*q2^2*q3 - sin(teta)*q2^2*q4 + 2*cos(phi)*cos(teta)*q2*q3^2 + 2*cos(phi)*cos(teta)*q2*q4^2 + cos(teta)*sin(phi)*q3^3 + sin(teta)*q3^2*q4 + cos(teta)*sin(phi)*q3*q4^2 + sin(teta)*q4^3))/(q1^2 + q2^2 + q3^2 + q4^2)^2,                             (2*rho*(sin(teta)*q1^3 - cos(teta)*sin(phi)*q1^2*q2 + 2*cos(phi)*cos(teta)*q1^2*q3 + sin(teta)*q1*q2^2 - sin(teta)*q1*q3^2 + 2*cos(teta)*sin(phi)*q1*q3*q4 + sin(teta)*q1*q4^2 - cos(teta)*sin(phi)*q2^3 + 2*cos(phi)*cos(teta)*q2^2*q3 + cos(teta)*sin(phi)*q2*q3^2 + 2*sin(teta)*q2*q3*q4 - cos(teta)*sin(phi)*q2*q4^2))/(q1^2 + q2^2 + q3^2 + q4^2)^2,                            -(2*rho*(cos(teta)*sin(phi)*q1^3 + sin(teta)*q1^2*q2 - 2*cos(phi)*cos(teta)*q1^2*q4 + cos(teta)*sin(phi)*q1*q2^2 + cos(teta)*sin(phi)*q1*q3^2 + 2*sin(teta)*q1*q3*q4 - cos(teta)*sin(phi)*q1*q4^2 + sin(teta)*q2^3 - 2*cos(phi)*cos(teta)*q2^2*q4 + sin(teta)*q2*q3^2 - 2*cos(teta)*sin(phi)*q2*q3*q4 - sin(teta)*q2*q4^2))/(q1^2 + q2^2 + q3^2 + q4^2)^2, 0, 0, 0, 0, 0, 0;
           0, 1, 0,                          -(2*rho*(- sin(teta)*q1^2*q2 + cos(phi)*cos(teta)*q1^2*q4 + 2*cos(teta)*sin(phi)*q1*q2^2 - 2*cos(phi)*cos(teta)*q1*q2*q3 - 2*sin(teta)*q1*q3*q4 + 2*cos(teta)*sin(phi)*q1*q4^2 + sin(teta)*q2^3 - cos(phi)*cos(teta)*q2^2*q4 + sin(teta)*q2*q3^2 + sin(teta)*q2*q4^2 - cos(phi)*cos(teta)*q3^2*q4 - cos(phi)*cos(teta)*q4^3))/(q1^2 + q2^2 + q3^2 + q4^2)^2,                           -(2*rho*(sin(teta)*q1^3 - 2*cos(teta)*sin(phi)*q1^2*q2 + cos(phi)*cos(teta)*q1^2*q3 - sin(teta)*q1*q2^2 + 2*cos(phi)*cos(teta)*q1*q2*q4 + sin(teta)*q1*q3^2 + sin(teta)*q1*q4^2 - cos(phi)*cos(teta)*q2^2*q3 - 2*cos(teta)*sin(phi)*q2*q3^2 - 2*sin(teta)*q2*q3*q4 + cos(phi)*cos(teta)*q3^3 + cos(phi)*cos(teta)*q3*q4^2))/(q1^2 + q2^2 + q3^2 + q4^2)^2,                            -(2*rho*(cos(phi)*cos(teta)*q1^2*q2 + sin(teta)*q1^2*q4 - 2*sin(teta)*q1*q2*q3 + 2*cos(phi)*cos(teta)*q1*q3*q4 + cos(phi)*cos(teta)*q2^3 + 2*cos(teta)*sin(phi)*q2^2*q3 + sin(teta)*q2^2*q4 - cos(phi)*cos(teta)*q2*q3^2 + cos(phi)*cos(teta)*q2*q4^2 - sin(teta)*q3^2*q4 + 2*cos(teta)*sin(phi)*q3*q4^2 + sin(teta)*q4^3))/(q1^2 + q2^2 + q3^2 + q4^2)^2,                             (2*rho*(cos(phi)*cos(teta)*q1^3 - sin(teta)*q1^2*q3 + 2*cos(teta)*sin(phi)*q1^2*q4 + cos(phi)*cos(teta)*q1*q2^2 + 2*sin(teta)*q1*q2*q4 + cos(phi)*cos(teta)*q1*q3^2 - cos(phi)*cos(teta)*q1*q4^2 - sin(teta)*q2^2*q3 + 2*cos(phi)*cos(teta)*q2*q3*q4 - sin(teta)*q3^3 + 2*cos(teta)*sin(phi)*q3^2*q4 + sin(teta)*q3*q4^2))/(q1^2 + q2^2 + q3^2 + q4^2)^2, 0, 0, 0, 0, 0, 0;
           0, 0, 1, -(2*rho*(cos(teta)*sin(phi)*q1^2*q2 - cos(phi)*cos(teta)*q1^2*q3 + 2*sin(teta)*q1*q2^2 - 2*cos(phi)*cos(teta)*q1*q2*q4 + 2*sin(teta)*q1*q3^2 - 2*cos(teta)*sin(phi)*q1*q3*q4 - cos(teta)*sin(phi)*q2^3 + cos(phi)*cos(teta)*q2^2*q3 - cos(teta)*sin(phi)*q2*q3^2 - cos(teta)*sin(phi)*q2*q4^2 + cos(phi)*cos(teta)*q3^3 + cos(phi)*cos(teta)*q3*q4^2))/(q1^2 + q2^2 + q3^2 + q4^2)^2, (2*rho*(cos(teta)*sin(phi)*q1^3 + 2*sin(teta)*q1^2*q2 - cos(phi)*cos(teta)*q1^2*q4 - cos(teta)*sin(phi)*q1*q2^2 + 2*cos(phi)*cos(teta)*q1*q2*q3 + cos(teta)*sin(phi)*q1*q3^2 + cos(teta)*sin(phi)*q1*q4^2 + cos(phi)*cos(teta)*q2^2*q4 + 2*cos(teta)*sin(phi)*q2*q3*q4 + 2*sin(teta)*q2*q4^2 - cos(phi)*cos(teta)*q3^2*q4 - cos(phi)*cos(teta)*q4^3))/(q1^2 + q2^2 + q3^2 + q4^2)^2, -(2*rho*(cos(phi)*cos(teta)*q1^3 - 2*sin(teta)*q1^2*q3 + cos(teta)*sin(phi)*q1^2*q4 + cos(phi)*cos(teta)*q1*q2^2 + 2*cos(teta)*sin(phi)*q1*q2*q3 - cos(phi)*cos(teta)*q1*q3^2 + cos(phi)*cos(teta)*q1*q4^2 + cos(teta)*sin(phi)*q2^2*q4 - 2*cos(phi)*cos(teta)*q2*q3*q4 - cos(teta)*sin(phi)*q3^2*q4 - 2*sin(teta)*q3*q4^2 + cos(teta)*sin(phi)*q4^3))/(q1^2 + q2^2 + q3^2 + q4^2)^2, -(2*rho*(cos(phi)*cos(teta)*q1^2*q2 + cos(teta)*sin(phi)*q1^2*q3 + 2*cos(teta)*sin(phi)*q1*q2*q4 - 2*cos(phi)*cos(teta)*q1*q3*q4 + cos(phi)*cos(teta)*q2^3 + cos(teta)*sin(phi)*q2^2*q3 + 2*sin(teta)*q2^2*q4 + cos(phi)*cos(teta)*q2*q3^2 - cos(phi)*cos(teta)*q2*q4^2 + cos(teta)*sin(phi)*q3^3 + 2*sin(teta)*q3^2*q4 - cos(teta)*sin(phi)*q3*q4^2))/(q1^2 + q2^2 + q3^2 + q4^2)^2, 0, 0, 0, 0, 0, 0];





% Gv= [1 0 -r*s;
%      0 1  r*c];
 
Gz = [ -(rho*cos(teta)*(- sin(phi)*q1^2 + 2*cos(phi)*q1*q4 - sin(phi)*q2^2 + 2*cos(phi)*q2*q3 + sin(phi)*q3^2 + sin(phi)*q4^2))/(q1^2 + q2^2 + q3^2 + q4^2),    (rho*(cos(phi)*sin(teta)*q1^2 + 2*cos(teta)*q1*q3 + 2*sin(phi)*sin(teta)*q1*q4 + cos(phi)*sin(teta)*q2^2 + 2*sin(phi)*sin(teta)*q2*q3 - 2*cos(teta)*q2*q4 - cos(phi)*sin(teta)*q3^2 - cos(phi)*sin(teta)*q4^2))/(q1^2 + q2^2 + q3^2 + q4^2), -(cos(phi)*cos(teta)*q1^2 - 2*sin(teta)*q1*q3 + 2*cos(teta)*sin(phi)*q1*q4 + cos(phi)*cos(teta)*q2^2 + 2*cos(teta)*sin(phi)*q2*q3 + 2*sin(teta)*q2*q4 - cos(phi)*cos(teta)*q3^2 - cos(phi)*cos(teta)*q4^2)/(q1^2 + q2^2 + q3^2 + q4^2);
             -(rho*cos(teta)*(cos(phi)*q1^2 + 2*sin(phi)*q1*q4 - cos(phi)*q2^2 - 2*sin(phi)*q2*q3 + cos(phi)*q3^2 - cos(phi)*q4^2))/(q1^2 + q2^2 + q3^2 + q4^2), -(rho*(- sin(phi)*sin(teta)*q1^2 + 2*cos(teta)*q1*q2 + 2*cos(phi)*sin(teta)*q1*q4 + sin(phi)*sin(teta)*q2^2 - 2*cos(phi)*sin(teta)*q2*q3 - sin(phi)*sin(teta)*q3^2 + 2*cos(teta)*q3*q4 + sin(phi)*sin(teta)*q4^2))/(q1^2 + q2^2 + q3^2 + q4^2), -(cos(teta)*sin(phi)*q1^2 + 2*sin(teta)*q1*q2 - 2*cos(phi)*cos(teta)*q1*q4 - cos(teta)*sin(phi)*q2^2 + 2*cos(phi)*cos(teta)*q2*q3 + cos(teta)*sin(phi)*q3^2 + 2*sin(teta)*q3*q4 - cos(teta)*sin(phi)*q4^2)/(q1^2 + q2^2 + q3^2 + q4^2);
                                              (2*rho*cos(teta)*(q1*q2*cos(phi) - q3*q4*cos(phi) + q1*q3*sin(phi) + q2*q4*sin(phi)))/(q1^2 + q2^2 + q3^2 + q4^2),                    (rho*(- cos(teta)*q1^2 - 2*sin(phi)*sin(teta)*q1*q2 + 2*cos(phi)*sin(teta)*q1*q3 + cos(teta)*q2^2 + 2*cos(phi)*sin(teta)*q2*q4 + cos(teta)*q3^2 + 2*sin(phi)*sin(teta)*q3*q4 - cos(teta)*q4^2))/(q1^2 + q2^2 + q3^2 + q4^2),                   -(sin(teta)*q1^2 - 2*cos(teta)*sin(phi)*q1*q2 + 2*cos(phi)*cos(teta)*q1*q3 - sin(teta)*q2^2 + 2*cos(phi)*cos(teta)*q2*q4 - sin(teta)*q3^2 + 2*cos(teta)*sin(phi)*q3*q4 + sin(teta)*q4^2)/(q1^2 + q2^2 + q3^2 + q4^2)];

 
%Gz= [c -r*s;
%    s  r*c];
     
% augment P
rng= len+1:len+3;
P(rng,rng)= Gv*P(1:13,1:13)*Gv' + Gz*R*Gz'; % feature cov
P(rng,1:13)= Gv*P(1:13,1:13); % vehicle to feature xcorr
P(1:13,rng)= P(rng,1:13)';
if len>13
    rnm= 14:len;
    P(rng,rnm)= Gv*P(1:13,rnm); % map to feature xcorr
    P(rnm,rng)= P(rng,rnm)';
end
