%% main
clear all;
params;

syms q0 q1 q2 q3 vx vy vz wx wy wz omegaL omegaR ... %m g b rho K_T...
    deltaL deltaR   ...
     u_jointT real; %...
    %Ix Iy Iz b_w K_deltaL K_deltaR K_omegaL K_omegaR ...
    %CL_A CL_B CL_C CD_A CD_B CD_C CLB_A CLB_B CDB_A CDB_B ...
    %SW Sw Sw_w CLdelta CDdelta Saw Sa yf zW xf xac Cwy ...
    %yt xW...
        


%inputs for FL. jointT is the thrust offset, externally set.
syms u_deltaA u_jointA u_deltaT real;
u = [u_deltaA u_jointA u_deltaT];
u_deltaL = u_jointA - u_deltaA/2;
u_deltaR = u_jointA + u_deltaA/2;
u_omegaL = u_jointT + u_deltaT/2;
u_omegaR = u_jointT - u_deltaT/2;

w = [wx;wy;wz];
v = [vx;vy;vz];
alpha = atan2(vz,vx);
Beta = atan2(vy,vx);

% Wing cord 
c = @(y) 0.28 - (0.28-0.09).*abs(y)./(p.b/2);
% Lift curves
CL = @(alpha) p.CL_A*sin(p.CL_B.*alpha)+p.CL_C; % These are wrong, just placeholders
CD = @(alpha) p.CD_A-p.CD_A.*cos(p.CD_B.*alpha)+p.CD_C;

% airflow in thrust wake
v_w_L = p.b_w .* omegaL; %[b_w .* omegaL; vy; vz];
v_w_R = p.b_w .* omegaR;

%Thrust
TL = p.K_T .* sign(omegaL).*omegaL.^2;
TR = p.K_T .* sign(omegaR).*omegaR.^2;

%Actuator dynamics
deltaLdot = p.K_deltaL .* (-deltaL + u_deltaL);
deltaRdot = p.K_deltaR .* (-deltaR + u_deltaR);
omegaLdot = p.K_omegaL .* (-omegaL + u_omegaL);
omegaRdot = p.K_omegaR .* (-omegaR + u_omegaR);

% Quaternion
qdot = 0.5.*[-q1 -q2 -q3; ...
            q0 -q3 q2; ...
            q3 q0 -q1; ...
            -q2 q1 q0]*w;
% Gravity
qc = quatconj([q0 q1 q2 q3]); %
qc0 = qc(1);
qc1 = qc(2);
qc2 = qc(3);
qc3 = qc(4);

gravity = p.g .* [2.*(qc1.*qc3 + qc0.*qc2); ...
                2.*(qc2.*qc3 - qc0.*qc1); ...
                (qc0.^2 - qc1.^2 - qc2.^2 + qc3.^2);
                zeros(3+4+4,1)];

% Inertial forces
inertialF = -[2*(vz.*wy - vy.*wz); ...
             2*(vx.*wz - vz.*wx); ...
             2*(vy.*wx - vx.*wy); ...
             (p.Iz - p.Iy).*wy.*wz./p.Ix; ...
             (p.Ix - p.Iz).*wx.*wz./p.Iy; ...
             (p.Iy - p.Ix).*wx.*wy./p.Iz; ...
             zeros(8,1)];

%y = [wx wy wz];

% Winglets
CLB = @(beta) p.CLB_A*sin(p.CLB_B.*beta); % These are wrong, just placeholders
CDB = @(beta) p.CDB_A-p.CDB_A.*cos(p.CDB_B.*beta);
L_W = -p.rho * sqrt(vx.^2+vy.^2) * p.SW * (CLB(Beta)*vx + CDB(Beta)*vy); % Lift in bodyframe, y
D_W = p.rho * sqrt(vx.^2+vy.^2) * p.SW * (-CLB(Beta)*vy + CDB(Beta)*vx); % Drag in bodyfram, x

% MAIN WING
% Lift inside wake
LLT = p.Sw_w * 0.5 * p.rho * abs(v_w_L) .* CL(0).*v_w_L ;
LRT = p.Sw_w * 0.5 * p.rho * abs(v_w_R) .* CL(0).*v_w_R ;
% Lift outside wake
LL = p.Sw * 0.5 * p.rho * sqrt(vx.^2 + vz.^2) .* ...
    (CL(alpha).*vx + CD(alpha).*vz);
LR = LL; % Yeah we -could- combine these, but too late!
% Drag inside wake
DLT = p.Sw_w * 0.5 * p.rho * abs(v_w_L) * CD(0) * v_w_L;
DRT = p.Sw_w * 0.5 * p.rho * abs(v_w_R) * CD(0) * v_w_R;
% Drag outside wake
DL = p.Sw * 0.5 * p.rho * sqrt(vx.^2  + vz.^2) .* ...
    (-CL(alpha).*vz + CD(alpha).*vx);
DR = DL; % Yeah, we -could- combine these as well...

% AILERONS
% Lift inside wake, signs bc rotation dir of aileron
FLT = -p.CLdelta * 0.5*p.rho*v_w_L.^2 * p.Saw * deltaL;
FRT = -p.CLdelta * 0.5*p.rho*v_w_R.^2 * p.Saw * deltaR;
% Lift outside wake
FL = -sign(vx).*p.CLdelta * 0.5*p.rho*(vx.^2 + vy.^2 + vz.^2) * p.Sa * deltaL;
FR = -sign(vx).*p.CLdelta * 0.5*p.rho*(vx.^2 + vy.^2 + vz.^2) * p.Sa * deltaR;
% Drag inside wake
DalphaLT = p.CDdelta * 0.5*p.rho*v_w_L.^2 * p.Saw * abs(deltaL);
DalphaRT = p.CDdelta * 0.5*p.rho*v_w_R.^2 * p.Saw * abs(deltaR);
% Drag outside wake, we add sign of vx because we might be reversing?
DalphaL = sign(vx).*p.CDdelta * 0.5*p.rho*(vx.^2 + vy.^2 + vz.^2) * p.Sa * abs(deltaL);
DalphaR = sign(vx).*p.CDdelta * 0.5*p.rho*(vx.^2 + vy.^2 + vz.^2) * p.Sa * abs(deltaR);

% f1
 % diff in ail deflection
  % inside wake
  % outside wake
 % resmom roll,
 % roll due to yaw rate
 % moment due to sideslip

% 3x2
 restmomroll = wx*[1 vx vx^2 vx^3]*sym('a%d%d', [4 3])*[1;vz;vz^2];

rollduetowz = wz*[1 vx vx^2]*sym('b%d%d', [3 4])*[1;vz;vz^2;vz^3];
f1 = p.yf.*(FLT+FL -FRT-FR) ...
    +restmomroll ...
    +rollduetowz ... 
    +p.zW*L_W;
% f2
 % joint ail deflection
  % inside wake
  % outside wake
 % pitch moment
 % restmom pitch rate, --- compensate for wake!
 % Pitch from winglets
f2 = p.xf.*(FLT+FL+FRT+FR) ...
    ... %+ 0.5*rho*Cmyl*(2*Sw.*(vx.^2+vy.^2+vz.^2)+Sw_w*(v_w_L.^2 + v_w_R.^2) ) ...
    -p.xac.*(LL+LR+LLT+LRT) ... % We do not assume Pith coeff static.
    -wy.*p.rho.*2.*p.Sw.*p.Cwy.*sqrt(vx.^2 + vz.^2) ...
    +p.zW*D_W;
% f3
 % diff in T
 % mom due to sideslip
 % restmom yaw rate, compensate for outside wake
restmomwz = wz*[1 vx vx^2 vx^3]*sym('c%d%d', [4 4])*[1;vz;vz^2;vz^3];

f3 = p.yt.*(TL-TR) ...
     -p.xW*L_W ...
     +restmomwz;
 
%velocities
vxdot = (TL +TR -DL -DR -DLT -DRT -D_W -DalphaL -DalphaR -DalphaLT -DalphaRT)./p.m;
vydot = L_W./p.m;
vzdot = (-LL -LR -LLT -LRT +FL +FR +FLT +FRT)./p.m;

% angular rates
wxdot = f1./p.Ix;
wydot = f2./p.Iy;
wzdot = f3./p.Iz;

%if any(abs(vxdot) > 100)
%    m
%end

states = [vx;vy;vz;wx;wy;wz;q0;q1;q2;q3;deltaL;deltaR;omegaL;omegaR];
statesdot = gravity + inertialF +  ...
    [vxdot;vydot;vzdot;wxdot;wydot;wzdot;qdot;...
    deltaLdot;deltaRdot;omegaLdot;omegaRdot];

vdot = statesdot(1:3);
wdot = statesdot(4:6);
qdot = statesdot(7:10);
deltaLdot = statesdot(11);
deltaRdot = statesdot(12);
omegaLdot = statesdot(13);
omegaRdot = statesdot(14);

%%
% Lets try to linearize around current state
syms vx0 vy0 vz0  wx0 wy0 wz0 q00 q10 q20 q30  deltaL0 deltaR0 omegaL0 omegaR0 u0_deltaA u0_jointA u0_deltaT real;
u = [u_deltaA u_jointA u_deltaT]';
u0 = [u0_deltaA u0_jointA u0_deltaT]';
JA = jacobian(statesdot,states);
JB = jacobian(statesdot,u);


wp = states;
wp(4:6) = [0;0;0]; % We are not rotating, this is our steady state?
wp(end-3:end) = [u_deltaL,u_deltaR,u_omegaL,u_omegaR]'; % The actuators have stopped moving anf are equal to their inputs.
 

A = subs(JA,states,wp);
B = subs(JB,u,u0);

xdotlin = A*(states-wp) + B*(u-u0);
eq = A*wp == -B*u0;
solve(eq(4:6),u0)

%%
u = [u_deltaA u_jointA u_deltaT]';
u0 = [u0_deltaA u0_jointA u0_deltaT]';

% We are here only interested in controlling w.
% At working point wdot = 0.
% We assume actuators to be instant and exact so replace those dynamics as
% direct inputs.
wp = [0,0,0]';
JA = jacobian(wdot,w);
JB = jacobian(subs(wdot,[deltaL,deltaR,omegaL,omegaR],[u_deltaL,u_deltaR,u_omegaL,u_omegaR]),u); % This we can do since we assume steady state, i.e. actuators not moving
A = subs(JA,w,wp);
B = subs(JB,w,wp);


wdotlin = A*(w-wp) + B*(u-u0);
eq = A*wp == -B*u0;
u0r = solve(eq,u0);


0.0 == u0_deltaT*( sign(0.5*u_deltaT + u_jointT)*(0.5*u_deltaT + u_jointT) + sign(0.5*u_deltaT - 1.0*u_jointT)*(0.5*u_deltaT - u_jointT) )

0.0 == - 1.0*u0_deltaA*(0.0504*sign(vx)*(vx^2 + vy^2 + vz^2) + 2.9e-5*(0.5*u_deltaT + u_jointT)^2 + 2.9e-5*(0.5*u_deltaT - 1.0*u_jointT)^2) ...
       - 1.0*u0_deltaT*(5.8e-5*(0.5*u_deltaA - 1.0*u_jointA)*(0.5*u_deltaT + u_jointT) + 5.8e-5*(0.5*u_deltaT - 1.0*u_jointT)*(0.5*u_deltaA + u_jointA))
 
 