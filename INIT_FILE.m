MAXP= 30*pi/180; % radians, maximum azimuth angle (-MAXT < thetta < MAXT)
RATEP= 20*pi/180; % rad/s, maximum rate of change in azimuth angle
MAXT= 30*pi/180; % radians, maximum elevation angle (-MAXP < psi < MAXP)
RATET= 20*pi/180; % rad/s, maximum rate of change in elevation angle
MAXR= 30*pi/180; % radians, maximum azimuth angle (-MAXT < thetta < MAXT)
RATER= 20*pi/180; % rad/s, maximum rate of change in azimuth angle
AT_WAYPOINT = 1;
<<<<<<< HEAD
dt=0.25;
DT_OBSERVE = dt;
=======
dt=0.5;
DT_OBSERVE = 10*dt;
>>>>>>> f8cd4fdc6d1a4ebcb15e715b29fb232ef7ac911a
SWITCH_CONTROL_NOISE = 1;
SWITCH_SENSOR_NOISE = 1;
SWITCH_INFLATE_NOISE = 0;
SWITCH_ASSOCIATION_KNOWN=0;
SWITCH_USE_IEKF = 0;
SWITCH_BATCH_UPDATE = 0;

GATE_REJECT = 10;
<<<<<<< HEAD
GATE_AUGMENT = 100;
=======
GATE_AUGMENT = 50;
>>>>>>> f8cd4fdc6d1a4ebcb15e715b29fb232ef7ac911a

wPsi=0;
wTeta=0;
wRoll=0;
dtsum=1;
Z=[];
rmax =110;
xtrue=[0,0,100,1,0,0,0];
iwp=1;
V=1;
fig=figure;
Q=zeros(4,4);
<<<<<<< HEAD
Q(1,1)=0.2^2;
Q(2,2)=(3*pi/180)^2;
Q(3,3)=(3*pi/180)^2;
Q(4,4)=(3*pi/180)^2;
P= diag([eps,eps,eps,eps,eps,eps,eps,eps,eps,eps,1e-6,1e-6,1e-6]);
=======
Q(1,1)=0.02^2;
Q(2,2)=(0.03*pi/180)^2;
Q(3,3)=(0.03*pi/180)^2;
Q(4,4)=(0.03*pi/180)^2;
P= diag([eps,eps,eps,eps,eps,eps,eps,1e-6,1e-6,1e-6,1e-6,1e-6,1e-6]);
>>>>>>> f8cd4fdc6d1a4ebcb15e715b29fb232ef7ac911a
% R = diag([eps,eps,eps]);
R= diag([3*pi/180^2,3*pi/180^2,2^2]);
x= [0,0,100,1,0,0,0,0,0,0,0,0,0]';
xnc= [0,0,100,1,0,0,0,0,0,0,0,0,0]';
Pnc= diag([eps,eps,eps,eps,eps,eps,eps,1e-6,1e-6,1e-6,1e-6,1e-6,1e-6]);