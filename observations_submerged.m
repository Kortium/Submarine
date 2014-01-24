function [z,idf]= observations_submerged(x, lm, idf, rmax)
%function [z,idf]= get_observations(x, lm, idf, rmax)
%
% INPUTS:
%   x - vehicle pose [x;y;z;q]
%   lm - set of all landmarks
%   idf - index tags for each landmark
%   rmax - maximum range of range-bearing sensor 
%
% OUTPUTS:
%   z - set of range-bearing observations
%   idf - landmark index tag for each observation
%
% Tim Bailey 2004.

[lm,idf]= get_visible_landmarks(x,lm,idf,rmax);
z= compute_range_bearing(x,lm);

%
%

function [lm,idf]= get_visible_landmarks(x,lm,idf,rmax)
% Select set of landmarks that are visible within vehicle's semi-circular field-of-view
dx= lm(1,:) - x(1);
dy= lm(2,:) - x(2);
dz= lm(3,:) - x(3);


[angle(1), angle(2), angle(3)] = quat2angle(x(4:7));
T = angle(1);
P = angle(2);

% incremental tests for bounding semi-circle
ii= find(abs(dx) < rmax & abs(dy) < rmax & abs(dz) < rmax ... % bounding cube
        & abs(T-atan2(dy,dx)) < pi/3 ...     % bounding line                     
        & (dx.^2 + dy.^2 + dz.^2) < rmax^2);           % bounding circle
  
%     & abs(P-atan2(dz,dx)-pi/6) < pi/3 ...
    
% Note: the bounding box test is unnecessary but illustrates a possible speedup technique
% as it quickly eliminates distant points. Ordering the landmark set would make this operation
% O(logN) rather that O(N).
  
lm= lm(:,ii);
idf= idf(ii);

%
%

function z= compute_range_bearing(x,lm)
% Compute exact observation
dx= lm(1,:) - x(1);
dy= lm(2,:) - x(2);
dz= lm(3,:) - x(3);
Rho = sqrt(dx.^2 + dy.^2 + dz.^2); 
[angle(1), angle(2), angle(3)] = quat2angle(x(4:7));
T = angle(1);
P = angle(2);
z= [atan2(dy,dx);%-T;
    atan2(dz,sqrt(dx.^2+dy.^2));% - P;
    Rho];
