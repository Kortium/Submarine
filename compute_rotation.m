function [wPsi,wTeta,wRoll,iwp]= compute_rotation(x, wp, iwp, minD, wPsi, ratePsi, maxPsi,  wTeta, rateTeta, maxTeta, wRoll, rateRoll, maxRoll, dt)

%
% INPUTS:
%   x - true position
%   wp - waypoints
%   iwp - index to current waypoint
%   minD - minimum distance to current waypoint before switching to next
%   Psi - current azimuth angle
%   ratePsi - max azimuth rate (rad/s)
%   maxPsi - max azimuth angle (rad)
%   Teta - current elevation angle
%   rateTeta - max elevation rate (rad/s)
%   maxTeta - max elevation angle (rad)
%   dt - timestep
%
% OUTPUTS:
%   Psi - new current azimuth angle
%   Teta - new current elevation angle
%   iwp - new current waypoint
%


% determine if current waypoint reached
cwp= wp(:,iwp);
d2= (cwp(1)-x(1))^2 + (cwp(2)-x(2))^2 + (cwp(3)-x(3))^2;
if d2 < minD^2
    iwp= iwp+1; % switch to next
    if iwp > size(wp,2) % reached final waypoint, flag and return
        iwp=0;
        return;
    end    
    cwp= wp(:,iwp); % next waypoint
end

% compute change in T to point towards current waypoint
[angle(1), angle(2), angle(3)] = quat2angle(x(4:7));
deltaPsi = pi_to_pi(atan2(cwp(2)-x(2), cwp(1)-x(1)) - angle(1) - wPsi);

DCM = angle2dcm_cc([-angle(1),0,0]);
dx(1) = cwp(1)-x(1);
dx(2) = cwp(2)-x(2);
dx(3) = cwp(3)-x(3);

dx = DCM*dx(1:3)';

persistent k;
if isempty(k)
    k=50;
end

if k==0
disp(dx)
k=50;
end
k=k-1;

deltaTeta = pi_to_pi(atan2(dx(3),sqrt((dx(1))^2+(dx(2))^2)) - angle(2) - wTeta);
deltaRoll = 0;%pi_to_pi(0-angle(3)-wRoll);



% limit rate
maxDelta= ratePsi*dt;
if abs(deltaPsi) > maxDelta
    deltaPsi= sign(deltaPsi)*maxDelta;
end

maxDelta= rateTeta*dt;
if abs(deltaTeta) > maxDelta
    deltaTeta= sign(deltaTeta)*maxDelta;
end

maxDelta= rateRoll*dt;
if abs(deltaRoll) > maxDelta
    deltaRoll= sign(deltaRoll)*maxDelta;
end

% limit angle
wPsi= wPsi+deltaPsi;
if abs(wPsi) > maxPsi
    wPsi= sign(wPsi)*maxPsi;
end

wTeta= wTeta+deltaTeta;
if abs(wTeta) > maxTeta
    wTeta= sign(wTeta)*maxTeta;
end

wRoll= wRoll+deltaRoll;
if abs(wRoll) > maxRoll
    wRoll= sign(wRoll)*maxRoll;
end