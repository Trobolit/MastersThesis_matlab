% All anynomous functions are moved into simulink simulator.
% They cannot be passed like this: as struct member.

load astredwedge.mat;
a(2).cdata(:,1) = 0;
a(2).cdata(:,2) = 1;
a(1).cdata(:,1) = 0;
a(1).cdata(:,3) = 1;

%Ixx = 2.8;
%Iyy = 0.3;
%Izz = 3;
% Waypoints: x,y,z, approach speed (speed to hold until the waypoint is
% reached), wp-radius ('hit sphere')

%{
p.waypoints =[ ...
    0,0,0,18;
    %0,0,-200,5;
    %0,0,-500,2;
    ...%0,0,-50,3;
    300,0,-10,12;
    310,0,-300,2;
    500,0,-300,18;
    500,300,-300,18;
    350,300,-300,18;
    350,0,-300,5;
    350,0,0,2;
    0,0,0,20;
    300,50,-500,17;
    1650,-700,-40,6;
    4000,1400,-40,18;
    3800+200, 5000,-40,18;
    7000,000,-40,18;
    350,850,-40,18;
    650,850,-40,18;
    100,700,-40,20];
%}
%{
p.waypoints =[ ...
    0,0,0,0.5;
    0,0,-100,1;
    0,100,-100,1;
    100,0,-100,7;
    500,500,-500,9;
    500,1500,-500,18;
    0,0,-500,16;
    0,0,-80,1;
    500,0,-80,18;
    600,500,-80,18;
    300,900,-80,20;
    0,400,-80,20;
    0,0,-80,8;
    0,0,0,0.5;
];
%}
%{
p.waypoints = [ ...
    0,0,0,1,1;
    0,0,-30,1,1;
    -50,50,-30,1,1;
    -100,50,-30,1,1;
    -110,50,-400,20,20;
    -1500,0,-600,18,30;
    -1000,1000,-500,18,20;
    1000,1000,-2000,14,30;
    1000,-100,-1000,10,20;
    1000,-100,-200,5,20;
    300,0,-200,10,16;
    50,0,-100,8,8;
    50,0,0,2,2;
    50,0,10,0.25,1
    ];
%}

p.waypoints = [ ...
    0,0,0,1,50;
    600,0,-800,18,50;
    1000,0,-800,18,50;
    1000,1200,-800,18,50;
    0,1200,-800,18,50;
    0,0,-800,15, 50
    ];

%p.wpradius = 20; % "hit sphere"
p.numwp = numel(p.waypoints(:,1));

% Simulation control frequency
p.Ts = 0.0025; %400 PixHawk rate; %0.05;% 20Hz COde indicated?
% Output log Ts
p.Ts_log = p.Ts; 

%speed of F.L. poles
p.wn = 3;

% Rigid body stuff
p.m=1.5; %used to be 0.75 b4 Avery
p.Ix=0.07; % kg*m^2
p.Iy=0.0079;
p.Iz=0.08;

p.I =  nan.*[p.Ix, 0, 0; 
          0, p.Iy, 0;
          0, 0, p.Iz];

p.g=9.82;
p.rho=1.225; %SI standard, sea level, 15 degC.

% Prop stuff
p.propd = 0.2286; % 9 inches
p.wakewidth = p.propd*0.8;
p.K_T=(2*p.rho*p.propd^3 * 0.5 / 48) * 0.1; %The last term is tuning factor, old sim used: %2*10^(-5);
p.K_v = (p.rho*p.propd*pi^2/2) * 0.05; % last part is tuning factor
p.b_w = sqrt(p.K_v/p.K_T) * 0.001; % last part is tuning factor, once we know cruising speed, rpm, prop eff, etc, this can be set.

%Aileron stuff
p.Sa=0.05*(0.29 + 0.14/2); %5cm wide, outside area, inside area but triangular so half. %10*0.025; % Area outside wake, one aileron

p.Saw=0.05*p.wakewidth; % 80% of propwidth; %10*0.0125;
p.CLdelta = 1; % Since delta always small, and aoa at aileron is small, this is the slope and the only thing needed.
p.CDdelta = 0.05; % These will the the "effectiveness" tuning factor.

%Winglet stuff
p.SW = 0.13*0.16; %0.04; % Guess, winglet area?

% Wing stuff, areas on one wing, not both
%total wing area: 0.67*0.27/2;
p.Sw_w = p.wakewidth*0.21;   % wing area in wake, guess
p.Sw = 0.16*0.26 + 0.3*0.14; %0.5;     % wing area outside wake, guess
p.b = 1.5;

%p.c = @(y) 0.28 - (0.28-0.09).*y./(p.b/2); % This should be pretty accurate

p.Cmyl = -0.01; % Wild guess, need to see what number here gives reasonable pitch down moment with speed. This will be this equations only tuning parameter. "l" is disregarded.
p.Cwy = 1;   % Also wild guess.
%distances
p.yf = 0.32; % This is correct when hovering. Avery claims it should remain when flying, but may be wrong.
p.yt = 0.33;
p.xW = 0.18;
p.xf=0.19;
p.xac=0.02; % averys guess, should maybe be fuction of alpha, but that might make it unstable??
p.zW = 0.02; % Measured, but strange value, might even be negative => unstable roll due to sideslip
p.yw = p.wakewidth;
% Integral eval point
p.yW = p.yt - p.yw/2 ; %distance to center wake minus half wake width
p.yWE = p.yt + p.yw/2; %distance to center wake plus half wake width
p.yWEE = p.b/2; %distance to end of wing.

%yac=0.38;
%yt=0.3;
%CL=0.4 * 0.01;
%CR=CL;
p.K_omegaL = 20;
p.K_omegaR = 20;
p.K_deltaL = 10;
p.K_deltaR = 10;

% Lift curves
%p.CL = @(alpha) 1.2*sin(2.*alpha); % These are wrong, just placeholders
%p.CD = @(alpha) 2-2.*cos(2.*alpha);
p.CL_A=1.2;
p.CL_B=2;
p.CL_C=0.2;
p.CD_A=2;
p.CD_B=2;
p.CD_C=0.05;
p.CLB_A=1.2;
p.CLB_B=2;
p.CDB_A=2;
p.CDB_B=2;

% Integral approximations
% a: restoring moment from roll, 3*2
p.a11 = -0.2735;
p.a12 = -0.0001295;
p.a13 = -0.04194;
p.a21 =  -0.01482;
p.a22 = -0.0009804;
p.a23 =  0.001643;
p.a31 =  -0.004045;
p.a32 = 3.699e-05;
p.a33 = 0;
p.a41 = 7.914e-05;
p.a42 = 0;
p.a43 = 0;

% Roll moment due to wz, 2*3 (vx*vz)
p.b11 = 0.02373;
p.b12 = 0.009625;
p.b13 = 0.0005311;
p.b14 = -0.0004559;
p.b21 = 0.006783;
p.b22 = 0.009343;
p.b23 = -3.616e-05;
p.b24 = 0;
p.b31 = 0.0002529;
p.b32 = -0.0003058;
p.b33 = 0;
p.b34 = 0;

% Redtoring moment, yaw rate, 3*3
p.c11 = -0.01891;
p.c12 = 0.0003808;
p.c13 = -0.005899;
p.c14 = -1.654e-05;
p.c21 = 0.001614;
p.c22 = 0.0008007;
p.c23 = 0.0003804;
p.c24 = 0;
p.c31 = 4.289e-05;
p.c32 = -3.022e-05;
p.c33 = 0;
p.c34 = 0;
p.c41 = -1.357e-05;
p.c42 = 0;
p.c43 = 0;
p.c44 = 0;






