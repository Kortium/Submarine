MAXP= 30*pi/180; % radians, maximum azimuth angle (-MAXT < thetta < MAXT)
RATEP= 20*pi/180; % rad/s, maximum rate of change in azimuth angle
MAXT= 30*pi/180; % radians, maximum elevation angle (-MAXP < psi < MAXP)
RATET= 20*pi/180; % rad/s, maximum rate of change in elevation angle
MAXR= 30*pi/180; % radians, maximum azimuth angle (-MAXT < thetta < MAXT)
RATER= 20*pi/180; % rad/s, maximum rate of change in azimuth angle
AT_WAYPOINT = 1;
dt=0.25;
DT_OBSERVE = dt;
SWITCH_CONTROL_NOISE = 1;
SWITCH_SENSOR_NOISE = 1;
SWITCH_INFLATE_NOISE = 0;
SWITCH_ASSOCIATION_KNOWN=0;
SWITCH_USE_IEKF = 0;
SWITCH_BATCH_UPDATE = 0;

GATE_REJECT = 10;
GATE_AUGMENT = 100;

wPsi=0;
wTeta=0;
wRoll=0;
dtsum=1;
Z=[];
rmax =150;
xtrue=[0,0,100,1,0,0,0];
iwp=1;
V=1;
fig=figure;
Q=zeros(4,4);
Q(1,1)=1^2;
Q(2,2)=(0.1*pi/180)^2;
Q(3,3)=(3*pi/180)^2;
Q(4,4)=(3*pi/180)^2;
P= diag([eps,eps,eps,eps,eps,eps,eps,eps,eps,eps,1e-6,1e-6,1e-6]);
% R = diag([eps,eps,eps]);
R= diag([3*pi/180^2,3*pi/180^2,2^2]);
x= [0,0,100,1,0,0,0,0,0,0,0,0,0]';
xnc= [0,0,100,1,0,0,0,0,0,0,0,0,0]';
Pnc= diag([eps,eps,eps,eps,eps,eps,eps,1e-6,1e-6,1e-6,1e-6,1e-6,1e-6]);