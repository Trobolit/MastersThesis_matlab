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
syms u_deltaA u_jointA u_deltaT real
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
TL = p.K_T .* omegaL.^2;
TR = p.K_T .* omegaR.^2;

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
%gravity = g .* [-2.*(q1.*q3 + q0.*q2); ...
%                -2.*(q2.*q3 - q0.*q1); ...
%                q0.^2 - q1.^2 - q2.^2 + q3.^2;
%                zeros(3+4+4,1)];
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

y = [wx wy wz];

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


%% Control law
y = [wx;wy;wz]; %wdot?
%ydot = wdot;
u = [u_deltaA u_jointA u_deltaT]';
% beta * inv(alpha) != 0 => good.

xdot = statesdot;
h = y;
f = simplify(subs(xdot,u,[0,0,0]'));
gn = subs(simplify(xdot-f),u,[1,1,1]');
clear g
%g = zeros(numel(states),numel(u));
g(11,1) = -p.K_deltaL/2;
g(11,2) = p.K_deltaL;
g(12,1) = p.K_deltaR/2;
g(12,2) = p.K_deltaR;
g(13,3) = p.K_omegaL/2;
g(14,3) = -p.K_omegaR/2;

simplify(xdot-(f+g*u) ); % check if the calucation is sorrect!

L_gh=jacobian(h,states)*g;
L_fh=jacobian(h,states)*f;
L_f_gh=jacobian(L_fh, states )*g; % M
L_f_fh=jacobian(L_fh, states )*f; % alpha

M = simplify(L_f_gh);
new_alpha = L_f_fh; % Which equals old alpha. NOT TRUE... ops.

syms wxr wyr wzr real
refs = [wxr wyr wzr]';


%% Control parameter setting

% The control law is shown in a pdf, but in short:
% ydotdot = alpha + M*u,
% u = inv(M)*(-L*xi + L_r - alpha)

%{
 - We differentiated to get the input as wanted.
 - Now we have to choose the coefficients in the second order system
    from every input to every output. We choose them do be decoupled.
 - L1 * y * u_ref
 - L2 * ydot
 - s^2 + L2*s + L1 :: linear system which we choose
%}

% We choose the system to have critical damping and a variable natural freq.
syms wn real;
%wn = 6; % higher no means faster system, uncomment if you want F.L.
%           directly, and not with wn as variable
L1 = wn^2;
L2 = 2*wn; %=> damp=1
Lwx = -[L1,L2]; %-[36,12];%-[1,2];
Lwy = Lwx; %-[1,2];
Lwz = Lwx; %-[1,2];
L = [Lwx, 0, 0, 0, 0; ...
    0, 0, Lwy, 0, 0; ...
    0, 0, 0, 0, Lwz];
Lrwx=L1; %36; %1;
Lrwy=Lrwx; %1;
Lrwz=Lrwx; %1;

Lr = [Lrwx, Lrwy, Lrwz]' .* refs;

xi = [wx; wxdot; wy; wydot; wz; wzdot];

%%
u_ref = M\(-L*xi + Lr - new_alpha);

childs = children(u_ref);
vpa(childs{1}(5),3)




