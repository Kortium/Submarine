function [T,P,R,iwp]= compute_azimuth_elevation_and_roll(x, wp, iwp, minD, T, rateT, maxT,  P, rateP, maxP, R, rateR, maxR, dt)

%
% INPUTS:
%   x - true position
%   wp - waypoints
%   iwp - index to current waypoint
%   minD - minimum distance to current waypoint before switching to next
%   T - current azimuth angle
%   rateT - max azimuth rate (rad/s)
%   maxT - max azimuth angle (rad)
%   P - current elevation angle
%   rateP - max elevation rate (rad/s)
%   maxP - max elevation angle (rad)
%   dt - timestep
%
% OUTPUTS:
%   T - new current elevation angle
%   P - new current azimuth angle
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
deltaT= pi_to_pi(atan2(cwp(2)-x(2), cwp(1)-x(1)) - angle(1) - T);
deltaP= pi_to_pi(-atan2( cwp(3)-x(3), sqrt((cwp(1)-x(1))^2+(cwp(2)-x(2))^2)) - angle(2) - P);
deltaR= pi_to_pi(-angle(3));

% limit rate
maxDelta= rateT*dt;
if abs(deltaT) > maxDelta
    deltaT= sign(deltaT)*maxDelta;
end

maxDelta= rateP*dt;
if abs(deltaP) > maxDelta
    deltaP= sign(deltaP)*maxDelta;
end

maxDelta= rateR*dt;
if abs(deltaR) > maxDelta
    deltaR= sign(deltaR)*maxDelta;
end

% limit angle
T= T+deltaT;
if abs(T) > maxT
    T= sign(T)*maxT;
end

P= P+deltaP;
if abs(P) > maxP
    P= sign(P)*maxP;
end

R= R+deltaR;
if abs(R) > maxR
    R= sign(R)*maxR;
end