%% main
clear all;

% Here we set output to q, and input as the four actuator states we can
% control.
% If my code is correct then M is not invertible here.

syms q0 q1 q2 q3 vx vy vz wx wy wz omegaL omegaR m g b rho K_T ...
    deltaL deltaR ...
    Ix Iy Iz b_w K_deltaL K_deltaR K_omegaL K_omegaR ...
    CL_A CL_B CL_C CD_A CD_B CD_C CLB_A CLB_B CDB_A CDB_B ...
    SW Sw Sw_w CLdelta CDdelta Saw Sa yf zW xf xac Cwy ...
    yt xW ...
    u_deltaL u_deltaR u_omegaL u_omegaR ...
    real;

w = [wx;wy;wz];
v = [vx;vy;vz];
alpha = atan2(vz,vx);
Beta = atan2(vy,vx);

% Wing cord 
c = @(y) 0.28 - (0.28-0.09).*abs(y)./(b/2);
% Lift curves
CL = @(alpha) CL_A*sin(CL_B.*alpha)+CL_C; % These are wrong, just placeholders
CD = @(alpha) CD_A-CD_A.*cos(CD_B.*alpha)+CD_C;

% airflow in thrust wake
v_w_L = b_w .* omegaL; %[b_w .* omegaL; vy; vz];
v_w_R = b_w .* omegaR;

%Thrust
TL = K_T .* omegaL.^2;
TR = K_T .* omegaR.^2;

%Actuator dynamics
deltaLdot = K_deltaL .* (-deltaL + u_deltaL);
deltaRdot = K_deltaR .* (-deltaR + u_deltaR);
omegaLdot = K_omegaL .* (-omegaL + u_omegaL);
omegaRdot = K_omegaR .* (-omegaR + u_omegaR);

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
gravity = g .* [2.*(qc1.*qc3 + qc0.*qc2); ...
                2.*(qc2.*qc3 - qc0.*qc1); ...
                (qc0.^2 - qc1.^2 - qc2.^2 + qc3.^2);
                zeros(3+4+4,1)];

% Inertial forces
inertialF = -[2*(vz.*wy - vy.*wz); ...
             2*(vx.*wz - vz.*wx); ...
             2*(vy.*wx - vx.*wy); ...
             (Iz - Iy).*wy.*wz./Ix; ...
             (Ix - Iz).*wx.*wz./Iy; ...
             (Iy - Ix).*wx.*wy./Iz; ...
             zeros(8,1)];

y = [wx wy wz];

% Winglets
CLB = @(beta) CLB_A*sin(CLB_B.*beta); % These are wrong, just placeholders
CDB = @(beta) CDB_A-CDB_A.*cos(CDB_B.*beta);
L_W = -rho * sqrt(vx.^2+vy.^2) * SW * (CLB(Beta)*vx + CDB(Beta)*vy); % Lift in bodyframe, y
D_W = rho * sqrt(vx.^2+vy.^2) * SW * (-CLB(Beta)*vy + CDB(Beta)*vx); % Drag in bodyfram, x

% MAIN WING
% Lift inside wake
LLT = Sw_w * 0.5 * rho * abs(v_w_L) .* CL(0).*v_w_L ;
LRT = Sw_w * 0.5 * rho * abs(v_w_R) .* CL(0).*v_w_R ;
% Lift outside wake
LL = Sw * 0.5 * rho * sqrt(vx.^2 + vz.^2) .* ...
    (CL(alpha).*vx + CD(alpha).*vz);
LR = LL; % Yeah we -could- combine these, but too late!
% Drag inside wake
DLT = Sw_w * 0.5 * rho * abs(v_w_L) * CD(0) * v_w_L;
DRT = Sw_w * 0.5 * rho * abs(v_w_R) * CD(0) * v_w_R;
% Drag outside wake
DL = Sw * 0.5 * rho * sqrt(vx.^2  + vz.^2) .* ...
    (-CL(alpha).*vz + CD(alpha).*vx);
DR = DL; % Yeah, we -could- combine these as well...

% AILERONS
% Lift inside wake, signs bc rotation dir of aileron
FLT = -CLdelta * 0.5*rho*v_w_L.^2 * Saw * deltaL;
FRT = -CLdelta * 0.5*rho*v_w_R.^2 * Saw * deltaR;
% Lift outside wake
FL = -sign(vx).*CLdelta * 0.5*rho*(vx.^2 + vy.^2 + vz.^2) * Sa * deltaL;
FR = -sign(vx).*CLdelta * 0.5*rho*(vx.^2 + vy.^2 + vz.^2) * Sa * deltaR;
% Drag inside wake
DalphaLT = CDdelta * 0.5*rho*v_w_L.^2 * Saw * abs(deltaL);
DalphaRT = CDdelta * 0.5*rho*v_w_R.^2 * Saw * abs(deltaR);
% Drag outside wake, we add sign of vx because we might be reversing?
DalphaL = sign(vx).*CDdelta * 0.5*rho*(vx.^2 + vy.^2 + vz.^2) * Sa * abs(deltaL);
DalphaR = sign(vx).*CDdelta * 0.5*rho*(vx.^2 + vy.^2 + vz.^2) * Sa * abs(deltaR);

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
f1 = yf.*(FLT+FL -FRT-FR) ...
    +restmomroll ...
    +rollduetowz ... 
    +zW*L_W;
% f2
 % joint ail deflection
  % inside wake
  % outside wake
 % pitch moment
 % restmom pitch rate, --- compensate for wake!
 % Pitch from winglets
f2 = xf.*(FLT+FL+FRT+FR) ...
    ... %+ 0.5*rho*Cmyl*(2*Sw.*(vx.^2+vy.^2+vz.^2)+Sw_w*(v_w_L.^2 + v_w_R.^2) ) ...
    -xac.*(LL+LR+LLT+LRT) ... % We do not assume Pith coeff static.
    -wy.*rho.*2.*Sw.*Cwy.*sqrt(vx.^2 + vz.^2) ...
    +zW*D_W;
% f3
 % diff in T
 % mom due to sideslip
 % restmom yaw rate, compensate for outside wake
restmomwz = wz*[1 vx vx^2 vx^3]*sym('c%d%d', [4 4])*[1;vz;vz^2;vz^3];

f3 = yt.*(TL-TR) ...
     -xW*L_W ...
     +restmomwz;
 
%velocities
vxdot = (TL +TR -DL -DR -DLT -DRT -D_W -DalphaL -DalphaR -DalphaLT -DalphaRT)./m;
vydot = L_W./m;
vzdot = (-LL -LR -LLT -LRT +FL +FR +FLT +FRT)./m;

% angular rates
wxdot = f1./Ix;
wydot = f2./Iy;
wzdot = f3./Iz;

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
y = [q0,q1,q2,q3]; %wdot?
%ydot = wdot;
u = [u_deltaL u_deltaR u_omegaL u_omegaR]';
% beta * inv(alpha) != 0 => good.

xdot = statesdot;
h = y;
f = subs(xdot,u,[0,0,0,0]');
gn = subs(simplify(xdot-f),u,[1,1,1,1]');
clear g
%g = zeros(numel(states),numel(u));
g(11,1) = K_deltaL;
g(12,2) = K_deltaR;
g(13,3) = K_omegaL;
g(14,4) = K_omegaR;
%deltaLdot = K_deltaL .* (-deltaL + u_deltaL);
%deltaRdot = K_deltaR .* (-deltaR + u_deltaR);
%omegaLdot = K_omegaL .* (-omegaL + u_omegaL);
%omegaRdot = K_omegaR .* (-omegaR + u_omegaR);

%g(11:14,1:3)=diag(gn(11:14));
simplify(xdot-(f+g*u) ); % check if the calucation is sorrect!

L_gh=jacobian(h,states)*g;
L_fh=jacobian(h,states)*f;
L_f_gh=jacobian(L_fh, states )*g; % M?
L_f_fh=jacobian(L_fh, states )*f; % alpha?

%L_f_g_gh = jacobian(L_f_gh, states )*g; % L_f_gh is just zeros.
%L_f_g_fh = jacobian(L_f_gh, states )*f;
L_f_f_gh = jacobian(L_f_fh, states )*g; % M?
L_f_f_fh = jacobian(L_f_fh, states )*f; % alpha?

M = L_f_f_gh;
new_alpha = L_f_f_fh;

%E=[ L_f_gh(1)*L_f_fh(1), L_f_gh(2)*L_f_fh(1), L_f_gh(3)*L_f_fh(1), L_f_gh(4)*L_f_fh(1) ; ...
%    L_f_gh(1)*L_f_fh(2), L_f_gh(2)*L_f_fh(2), L_f_gh(3)*L_f_fh(2), L_f_gh(4)*L_f_fh(2) ; ...
%    L_f_gh(1)*L_f_fh(3), L_f_gh(2)*L_f_fh(3), L_f_gh(3)*L_f_fh(3), L_f_gh(4)*L_f_fh(3) ; ...
%    L_f_gh(1)*L_f_fh(4), L_f_gh(2)*L_f_fh(4), L_f_gh(3)*L_f_fh(4), L_f_gh(4)*L_f_fh(4) ];

syms q0r q1r q2r q3r real
refs = [q0r q1r q2r q3r]';


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

%L1 = diag(ones(4,1));
%L2 = 2*diag(ones(4,1));

% out/in = k/f(s) => out*f(s) = in*k
syms slowestpole real;
slowestpole = 8; % higher no means faster system
a=slowestpole;
b=2*a;
c=4*a;
L1 = a*b*c; % Constant term
L2 = a*b+c*(a+b); % s term => damp=1
L3 = (a+b+c); % s^2 term
% s^3 term is 1.

% (s-a)*(s-b)*(s-c) = 
% s^3 -(a+b+c)*s^2 + (a*b+c*(a+b))*s -a*b*c
% expand(subs((s+a)*(s+b)*(s+c), [a,b,c],[2,3,100]))
% s^3 + 308*s^2 + 22400*s + 160000 => less than 1 sec step, ~0.5
Lq0 = -[L1,L2,L3]; %-[36,12];%-[1,2];
Lq1 = Lq0; %-[1,2];
Lq2 = Lq0; %-[1,2];
Lq3 = Lq0;
L = [Lq0, 0, 0, 0, 0, 0, 0, 0, 0, 0; ...
    0, 0, 0, Lq1, 0, 0, 0, 0, 0, 0; ...
    0, 0, 0, 0, 0, 0, Lq2, 0, 0, 0; ...
    0, 0, 0, 0, 0, 0, 0, 0, 0, Lq3];
Lrq0=L1;
Lrq1=L1;
Lrq2=L1; 
Lrq3=L1;

Lr = [Lrq0 Lrq1 Lrq2 Lrq3]' .* refs;

%xi = [wx; wxdot; wy; wydot; wz; wzdot];
qddot = jacobian(qdot,states)*statesdot;
q0dot = qdot(1);
q1dot = qdot(2);
q2dot = qdot(3);
q3dot = qdot(4);
q0ddot = qddot(1);
q1ddot = qddot(2);
q2ddot = qddot(3);
q3ddot = qddot(4);
xi = [q0 q0dot q0ddot q1 q1dot q1ddot q2 q2dot q2ddot q3 q3dot q3ddot]';

%%
%u_ref = inv(M)*(-L*xi + Lr - alpha)
u_ref = M\(-L*xi + Lr - new_alpha);


%%
% Check invertibility
%Mdet = det(M); % use cond
Msimp = simplify(M);
Mvsimp = vpa(Msimp,3);
Minv = myinv(M);
part2 = vpa((-L*xi + Lr - alpha),3);
u_ref = Minv*part2;



