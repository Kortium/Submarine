function [Z,H]= observe_model(X, idf)
%function [z,H]= observe_model(x, idf)
%
% INPUTS:
%   x - state vector
%   idf - index of feature order in state
%
% OUTPUTS:
%   z - predicted observation
%   H - observation Jacobian
%
% Given a feature index (ie, the order of the feature in the state vector),
% predict the expected range-bearing observation of this feature and its Jacobian.
%
% Tim Bailey 2004.

Nxv= 13; % number of vehicle pose states
fpos= Nxv + idf*3 - 2; % position of xf in state
H= zeros(3, length(X));

% auxiliary values
% dx= x(fpos)  -x(1); 
% dy= x(fpos+1)-x(2);
% dz= x(fpos+2)-x(3);
x=X(1);
y=X(2);
z=X(3);

xf = X(fpos);
yf = X(fpos+1);
zf = X(fpos+2);

% qsum = x(4)^2+x(5)^2+x(6)^2+x(7)^2;
% qmod = sqrt(qsum);
% d2= dx^2 + dy^2 + dz^2;
% d= sqrt(d2);
% xd= dx/d;
% yd= dy/d;
% zd= dz/d;
% xd2= dx/d2;
% yd2= dy/d2;
% zd2= dz/d2;

% [angle1,angle2,~] = quat2angle(X(4:7)');
% predict z
% Z= [atan2(yf  -y,xf-x) - angle1;
%       asin((zf  -z)/sqrt((xf  -x)^2+(yf  -y)^2+(zf  -z)^2)) - angle2;
%       sqrt((xf  -x)^2+(yf  -y)^2+(zf  -z)^2)];

dx= xf  -x;
dy= yf  -y;
dz= zf  -z;
Rho = sqrt(dx^2 + dy^2 + dz^2); 

Z= [atan2(dy,dx);
      atan2(dz,sqrt(dx^2+dy^2));
      Rho];

% calculate H
% H(:,1:3) = [-xd -yd 0; yd2 -xd2 -1];
% H(:,fpos:fpos+1)= [ xd  yd;  -yd2  xd2];

H(:,1:13) = [                                                    -(y - yf)/((x - xf)^2*((y - yf)^2/(x - xf)^2 + 1)),                                                              1/((x - xf)*((y - yf)^2/(x - xf)^2 + 1)),                                                                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
 ((x - xf)*(z - zf))/(((x - xf)^2 + (y - yf)^2)^(1/2)*((x - xf)^2 + (y - yf)^2 - 2*z*zf + z^2 + zf^2)), ((y - yf)*(z - zf))/(((x - xf)^2 + (y - yf)^2)^(1/2)*((x - xf)^2 + (y - yf)^2 - 2*z*zf + z^2 + zf^2)), -1/(((x - xf)^2 + (y - yf)^2)^(1/2)*((z - zf)^2/((x - xf)^2 + (y - yf)^2) + 1)), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                                                 (x - xf)/((x - xf)^2 + (y - yf)^2 + (z - zf)^2)^(1/2),                                                 (y - yf)/((x - xf)^2 + (y - yf)^2 + (z - zf)^2)^(1/2),                           (z - zf)/((x - xf)^2 + (y - yf)^2 + (z - zf)^2)^(1/2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

 
H(:,fpos:fpos+2)=[                                                       (y - yf)/((x - xf)^2*((y - yf)^2/(x - xf)^2 + 1)),                                                               -1/((x - xf)*((y - yf)^2/(x - xf)^2 + 1)),                                                                              0;
 -((2*x - 2*xf)*(z - zf))/(2*((x - xf)^2 + (y - yf)^2)^(3/2)*((z - zf)^2/((x - xf)^2 + (y - yf)^2) + 1)), -((2*y - 2*yf)*(z - zf))/(2*((x - xf)^2 + (y - yf)^2)^(3/2)*((z - zf)^2/((x - xf)^2 + (y - yf)^2) + 1)), 1/(((x - xf)^2 + (y - yf)^2)^(1/2)*((z - zf)^2/((x - xf)^2 + (y - yf)^2) + 1));
                                          -(2*x - 2*xf)/(2*((x - xf)^2 + (y - yf)^2 + (z - zf)^2)^(1/2)),                                          -(2*y - 2*yf)/(2*((x - xf)^2 + (y - yf)^2 + (z - zf)^2)^(1/2)),                 -(2*z - 2*zf)/(2*((x - xf)^2 + (y - yf)^2 + (z - zf)^2)^(1/2))];

                                                   
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
% [                                                                       -(abs(y - yf)*sign(x - xf))/(abs(x - xf)^2 + abs(y - yf)^2),                                                                        (abs(x - xf)*sign(y - yf))/(abs(x - xf)^2 + abs(y - yf)^2),                                                                                                                                                        0,                                                                  (q2 + q4)/(q1^2 - 2*q1*q3 + q2^2 + 2*q2*q4 + q3^2 + q4^2) - (q2 - q4)/(q1^2 + 2*q1*q3 + q2^2 - 2*q2*q4 + q3^2 + q4^2),                                                               (q1 + q3)/(q1^2 + 2*q1*q3 + q2^2 - 2*q2*q4 + q3^2 + q4^2) - (q1 - q3)/(q1^2 - 2*q1*q3 + q2^2 + 2*q2*q4 + q3^2 + q4^2),                                                              - (q2 + q4)/(q1^2 - 2*q1*q3 + q2^2 + 2*q2*q4 + q3^2 + q4^2) - (q2 - q4)/(q1^2 + 2*q1*q3 + q2^2 - 2*q2*q4 + q3^2 + q4^2),                                                             - (q1 + q3)/(q1^2 + 2*q1*q3 + q2^2 - 2*q2*q4 + q3^2 + q4^2) - (q1 - q3)/(q1^2 - 2*q1*q3 + q2^2 + 2*q2*q4 + q3^2 + q4^2), 0, 0, 0, 0, 0, 0]                                                        
% [ -((x - xf)*(z - zf))/((1 - (z - zf)^2/((x - xf)^2 + (y - yf)^2 + (z - zf)^2))^(1/2)*((x - xf)^2 + (y - yf)^2 + (z - zf)^2)^(3/2)), -((y - yf)*(z - zf))/((1 - (z - zf)^2/((x - xf)^2 + (y - yf)^2 + (z - zf)^2))^(1/2)*((x - xf)^2 + (y - yf)^2 + (z - zf)^2)^(3/2)), (x^2 - 2*x*xf + xf^2 + y^2 - 2*y*yf + yf^2)/((1 - (z - zf)^2/((x - xf)^2 + (y - yf)^2 + (z - zf)^2))^(1/2)*((x - xf)^2 + (y - yf)^2 + (z - zf)^2)^(3/2)), -(2*(- q1^2*q3 + 2*q1*q2*q4 + q2^2*q3 + q3^3 + q3*q4^2))/((1 - ((2*q1*q3)/(q1^2 + q2^2 + q3^2 + q4^2) - (2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2))^2)^(1/2)*(q1^2 + q2^2 + q3^2 + q4^2)^2), (2*(q1^2*q4 + 2*q1*q2*q3 - q2^2*q4 + q3^2*q4 + q4^3))/((1 - ((2*q1*q3)/(q1^2 + q2^2 + q3^2 + q4^2) - (2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2))^2)^(1/2)*(q1^2 + q2^2 + q3^2 + q4^2)^2), -(2*(q1^3 + q1*q2^2 - q1*q3^2 + q1*q4^2 + 2*q2*q3*q4))/((1 - ((2*q1*q3)/(q1^2 + q2^2 + q3^2 + q4^2) - (2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2))^2)^(1/2)*(q1^2 + q2^2 + q3^2 + q4^2)^2), (2*(q1^2*q2 + 2*q1*q3*q4 + q2^3 + q2*q3^2 - q2*q4^2))/((1 - ((2*q1*q3)/(q1^2 + q2^2 + q3^2 + q4^2) - (2*q2*q4)/(q1^2 + q2^2 + q3^2 + q4^2))^2)^(1/2)*(q1^2 + q2^2 + q3^2 + q4^2)^2), 0, 0, 0, 0, 0, 0]
% [                                                                             (x - xf)/((x - xf)^2 + (y - yf)^2 + (z - zf)^2)^(1/2),                                                                             (y - yf)/((x - xf)^2 + (y - yf)^2 + (z - zf)^2)^(1/2),                                                                                                    (z - zf)/((x - xf)^2 + (y - yf)^2 + (z - zf)^2)^(1/2),                                                                                                                                                                                      0,                                                                                                                                                                                   0,                                                                                                                                                                                    0,                                                                                                                                                                                   0, 0, 0, 0, 0, 0, 0]