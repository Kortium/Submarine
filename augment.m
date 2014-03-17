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


Gv = dGdX(phi,rho,teta,q1,q2,q3,q4);




% Gv= [1 0 -r*s;
%      0 1  r*c];
 
Gz = dGdZ(phi,rho,teta,q1,q2,q3,q4);
 
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
