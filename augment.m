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
teta= z(1); ksi = z(2); rho= z(3);

q1 = x(4);
q2 = x(5);
q3 = x(6);
q4 = x(7);
% 
[angle(1), angle(2), ~] = quat2angle(x(4:7)');
% augment x

x= [x;
    x(1) + rho*cos(ksi+angle(2))*cos(teta+angle(1));
    x(2) + rho*cos(ksi+angle(2))*sin(teta+angle(1));
    x(3) + rho*sin(ksi+angle(2))];

% jacobians

% Gv = [ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
%            0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
%            0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];


Gv = [ 1, 0, 0,   (2*rho*sin(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*cos(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2)))*(q1^2*q4 + 2*q1*q2*q3 - q2^2*q4 + q3^2*q4 + q4^3))/(q1^4 + 2*q1^2*q2^2 - 2*q1^2*q3^2 + 2*q1^2*q4^2 + 8*q1*q2*q3*q4 + q2^4 + 2*q2^2*q3^2 - 2*q2^2*q4^2 + q3^4 + 2*q3^2*q4^2 + q4^4) - (2*rho*cos(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*sin(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2)))*(- q1^2*q3 + 2*q1*q2*q4 + q2^2*q3 + q3^3 + q3*q4^2))/((1 - (4*(q1*q3 - q2*q4)^2)/(q1^2 + q2^2 + q3^2 + q4^2)^2)^(1/2)*(q1^2 + q2^2 + q3^2 + q4^2)^2), (2*rho*sin(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*cos(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2)))*(- q1^2*q3 + 2*q1*q2*q4 + q2^2*q3 + q3^3 + q3*q4^2))/(q1^4 + 2*q1^2*q2^2 - 2*q1^2*q3^2 + 2*q1^2*q4^2 + 8*q1*q2*q3*q4 + q2^4 + 2*q2^2*q3^2 - 2*q2^2*q4^2 + q3^4 + 2*q3^2*q4^2 + q4^4) + (2*rho*cos(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*sin(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2)))*(q1^2*q4 + 2*q1*q2*q3 - q2^2*q4 + q3^2*q4 + q4^3))/((1 - (4*(q1*q3 - q2*q4)^2)/(q1^2 + q2^2 + q3^2 + q4^2)^2)^(1/2)*(q1^2 + q2^2 + q3^2 + q4^2)^2), - (2*rho*sin(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*cos(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2)))*(q1^2*q2 + 2*q1*q3*q4 + q2^3 + q2*q3^2 - q2*q4^2))/(q1^4 + 2*q1^2*q2^2 - 2*q1^2*q3^2 + 2*q1^2*q4^2 + 8*q1*q2*q3*q4 + q2^4 + 2*q2^2*q3^2 - 2*q2^2*q4^2 + q3^4 + 2*q3^2*q4^2 + q4^4) - (2*rho*cos(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*sin(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2)))*(q1^3 + q1*q2^2 - q1*q3^2 + q1*q4^2 + 2*q2*q3*q4))/((1 - (4*(q1*q3 - q2*q4)^2)/(q1^2 + q2^2 + q3^2 + q4^2)^2)^(1/2)*(q1^2 + q2^2 + q3^2 + q4^2)^2), (2*rho*cos(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*sin(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2)))*(q1^2*q2 + 2*q1*q3*q4 + q2^3 + q2*q3^2 - q2*q4^2))/((1 - (4*(q1*q3 - q2*q4)^2)/(q1^2 + q2^2 + q3^2 + q4^2)^2)^(1/2)*(q1^2 + q2^2 + q3^2 + q4^2)^2) - (2*rho*sin(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*cos(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2)))*(q1^3 + q1*q2^2 - q1*q3^2 + q1*q4^2 + 2*q2*q3*q4))/(q1^4 + 2*q1^2*q2^2 - 2*q1^2*q3^2 + 2*q1^2*q4^2 + 8*q1*q2*q3*q4 + q2^4 + 2*q2^2*q3^2 - 2*q2^2*q4^2 + q3^4 + 2*q3^2*q4^2 + q4^4), 0, 0, 0, 0, 0, 0;
           0, 1, 0, - (2*rho*cos(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*cos(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2)))*(q1^2*q4 + 2*q1*q2*q3 - q2^2*q4 + q3^2*q4 + q4^3))/(q1^4 + 2*q1^2*q2^2 - 2*q1^2*q3^2 + 2*q1^2*q4^2 + 8*q1*q2*q3*q4 + q2^4 + 2*q2^2*q3^2 - 2*q2^2*q4^2 + q3^4 + 2*q3^2*q4^2 + q4^4) - (2*rho*sin(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*sin(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2)))*(- q1^2*q3 + 2*q1*q2*q4 + q2^2*q3 + q3^3 + q3*q4^2))/((1 - (4*(q1*q3 - q2*q4)^2)/(q1^2 + q2^2 + q3^2 + q4^2)^2)^(1/2)*(q1^2 + q2^2 + q3^2 + q4^2)^2), (2*rho*sin(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*sin(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2)))*(q1^2*q4 + 2*q1*q2*q3 - q2^2*q4 + q3^2*q4 + q4^3))/((1 - (4*(q1*q3 - q2*q4)^2)/(q1^2 + q2^2 + q3^2 + q4^2)^2)^(1/2)*(q1^2 + q2^2 + q3^2 + q4^2)^2) - (2*rho*cos(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*cos(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2)))*(- q1^2*q3 + 2*q1*q2*q4 + q2^2*q3 + q3^3 + q3*q4^2))/(q1^4 + 2*q1^2*q2^2 - 2*q1^2*q3^2 + 2*q1^2*q4^2 + 8*q1*q2*q3*q4 + q2^4 + 2*q2^2*q3^2 - 2*q2^2*q4^2 + q3^4 + 2*q3^2*q4^2 + q4^4),   (2*rho*cos(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*cos(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2)))*(q1^2*q2 + 2*q1*q3*q4 + q2^3 + q2*q3^2 - q2*q4^2))/(q1^4 + 2*q1^2*q2^2 - 2*q1^2*q3^2 + 2*q1^2*q4^2 + 8*q1*q2*q3*q4 + q2^4 + 2*q2^2*q3^2 - 2*q2^2*q4^2 + q3^4 + 2*q3^2*q4^2 + q4^4) - (2*rho*sin(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*sin(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2)))*(q1^3 + q1*q2^2 - q1*q3^2 + q1*q4^2 + 2*q2*q3*q4))/((1 - (4*(q1*q3 - q2*q4)^2)/(q1^2 + q2^2 + q3^2 + q4^2)^2)^(1/2)*(q1^2 + q2^2 + q3^2 + q4^2)^2), (2*rho*cos(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*cos(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2)))*(q1^3 + q1*q2^2 - q1*q3^2 + q1*q4^2 + 2*q2*q3*q4))/(q1^4 + 2*q1^2*q2^2 - 2*q1^2*q3^2 + 2*q1^2*q4^2 + 8*q1*q2*q3*q4 + q2^4 + 2*q2^2*q3^2 - 2*q2^2*q4^2 + q3^4 + 2*q3^2*q4^2 + q4^4) + (2*rho*sin(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*sin(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2)))*(q1^2*q2 + 2*q1*q3*q4 + q2^3 + q2*q3^2 - q2*q4^2))/((1 - (4*(q1*q3 - q2*q4)^2)/(q1^2 + q2^2 + q3^2 + q4^2)^2)^(1/2)*(q1^2 + q2^2 + q3^2 + q4^2)^2), 0, 0, 0, 0, 0, 0;
           0, 0, 1,                                                                                                                                                                                                                                                                                                                                                                                                    (2*rho*cos(ksi + asin((2*(q1*q3 - q2*q4))/(q1^2 + q2^2 + q3^2 + q4^2)))*(- q1^2*q3 + 2*q1*q2*q4 + q2^2*q3 + q3^3 + q3*q4^2))/((1 - (4*(q1*q3 - q2*q4)^2)/(q1^2 + q2^2 + q3^2 + q4^2)^2)^(1/2)*(q1^2 + q2^2 + q3^2 + q4^2)^2),                                                                                                                                                                                                                                                                                                                                                                                                   -(2*rho*cos(ksi + asin((2*(q1*q3 - q2*q4))/(q1^2 + q2^2 + q3^2 + q4^2)))*(q1^2*q4 + 2*q1*q2*q3 - q2^2*q4 + q3^2*q4 + q4^3))/((1 - (4*(q1*q3 - q2*q4)^2)/(q1^2 + q2^2 + q3^2 + q4^2)^2)^(1/2)*(q1^2 + q2^2 + q3^2 + q4^2)^2),                                                                                                                                                                                                                                                                                                                                                                                                    (2*rho*cos(ksi + asin((2*(q1*q3 - q2*q4))/(q1^2 + q2^2 + q3^2 + q4^2)))*(q1^3 + q1*q2^2 - q1*q3^2 + q1*q4^2 + 2*q2*q3*q4))/((1 - (4*(q1*q3 - q2*q4)^2)/(q1^2 + q2^2 + q3^2 + q4^2)^2)^(1/2)*(q1^2 + q2^2 + q3^2 + q4^2)^2),                                                                                                                                                                                                                                                                                                                                                                                                 -(2*rho*cos(ksi + asin((2*(q1*q3 - q2*q4))/(q1^2 + q2^2 + q3^2 + q4^2)))*(q1^2*q2 + 2*q1*q3*q4 + q2^3 + q2*q3^2 - q2*q4^2))/((1 - (4*(q1*q3 - q2*q4)^2)/(q1^2 + q2^2 + q3^2 + q4^2)^2)^(1/2)*(q1^2 + q2^2 + q3^2 + q4^2)^2), 0, 0, 0, 0, 0, 0];





% Gv= [1 0 -r*s;
%      0 1  r*c];
 
Gz = [ -rho*sin(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*cos(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2))), -rho*cos(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*sin(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2))), cos(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*cos(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2)));
            rho*cos(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*cos(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2))), -rho*sin(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*sin(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2))), sin(teta + atan2((2*q1*q4 + 2*q2*q3),(q1^2 + q2^2 - q3^2 - q4^2)))*cos(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2)));
                                                                                                                                                 0,                                                                    rho*cos(ksi + asin((2*q1*q3 - 2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2))),                                                                   sin(ksi + asin((2*(q1*q3 - q2*q4))/(q1^2 + q2^2 + q3^2 + q4^2)))];

 
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
