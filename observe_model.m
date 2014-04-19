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

q1=X(4);
q2=X(5);
q3=X(6);
q4=X(7);

xf = X(fpos);
yf = X(fpos+1);
zf = X(fpos+2);

% predict z

dx= x-xf;
dy= y-yf;
dz= z-zf;
Rho = sqrt(dx^2 + dy^2 + dz^2); 

DCM = quat2dcm_cc([X(4) X(5) X(6) X(7)]);

cfc = DCM*[dx;dy;dz];
% cfc = [dx;dy;dz];
Z = [pi_to_pi(atan2(cfc(2),cfc(1)));
            pi_to_pi(atan2(cfc(3),sqrt(cfc(1)^2+cfc(2)^2)));
            Rho];
        
% calculate H
% H(:,1:13) = dHdX(x,xf,y,yf,z,zf);
H(:,1:13) = dHdX(q1,q2,q3,q4,x,xf,y,yf,z,zf);
% H(:,fpos:fpos+2) = dHdf(x,xf,y,yf,z,zf);
H(:,fpos:fpos+2) = dHdf(q1,q2,q3,q4,x,xf,y,yf,z,zf);

                                                   
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
