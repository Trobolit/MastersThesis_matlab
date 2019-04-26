%% main
clear all;

syms q0 q1 q2 q3 vx vy vz wx wy wz WL WR DL DR K m g rho OmegaL OmegaR ...
    deltaL deltaR udeltaR udeltaL uWR uWL Sa Sw b CL CR yac xac yt ...
    Ix Iy Iz xf v1 v2 v3 v4 real

%Actuator dynamics
WLdot = OmegaL*(-WL + uWL);
WRdot = OmegaR*(-WR + uWR);
deltaLdot = DL*(-deltaL + udeltaL);
deltaRdot = DR*(-deltaR + udeltaR);

% Rigid body dynamics
vxdot = K*(WL^2+WR^2)/m + 2*(q1*q3+q0*q2)*g - vz*wy + vy*wz;
vydot = 2*(q2*q3 - q0*q1)*g - vx*wz + vz*wx;
vzdot = rho*Sa*b^2*(WL^2*deltaL + WR^2*deltaR)/(2*m) ...
    - rho*b^2*(WL^2+WR^2)*Sw*(CL+CR)/(2*m) ...
    + g*(q0^2 - q1^2 -q2^2 +q3^2) ...
    - vy*wx + vx*wy;
vdot = [vxdot, vydot, vzdot]';

wxdot = 0.5*rho*b^2*(Sa*(WR^2*deltaR-WL^2*deltaL)*yac ...
    + Sw*(WL^2 - WR^2)*yt)/Ix ...
    - (Iz-Iy)*wy*wz/Ix;
wydot = 0.5*rho*b^2*(Sa*(WL^2*deltaL + WR^2*deltaR)*xf ...
    - Sw*(CL*WL^2+CR*WR^2)*xac)/Iy ...
    - (Ix-Iz)*wx*wz/Iy;
wzdot = K*(WL^2 - WR^2)*yt/Iz - (Iy-Ix)*wx*wy/Iz;
wdot = [wxdot, wydot, wzdot]';

q0dot = 0.5*(q1*wx + q2*wy + q3*wz);
q1dot = 0.5*(q2*wz - q3*wy - q0*wx);
q2dot = 0.5*(q3*wx - q0*wy - q1*wz);
q3dot = 0.5*(q1*wy - q2*wx - q0*wz);
qdot = [q0dot,q1dot,q2dot,q3dot]';

% States, state space equations (using above equations) and input declarations
states = [vx vy vz wx wy wz q0 q1 q2 q3 WL WR deltaL deltaR]';
statesdot = [vdot; wdot; qdot; WLdot; WRdot; deltaLdot; deltaRdot];
%inputs = [uWL-uWR, udeltaL, udeltaR]';
ydot = [wdot; vzdot];
y = [wx, wy, wz, vx]';

% Start differentiating, remember that y=[v,w] only.
% ydot = [vdot, wdot] which are driectly from the state space equation, but
% still does not contain any input, so we differentiate that one more time.
% Lets do all states 
ydotdot = expand(jacobian(ydot, states)*statesdot);
%J_full = jacobian(statesdot, states)*statesdot';

%collect(J,inputs(1))

for i=1:numel(ydotdot)
    C1{i} = coeffs(ydotdot(i), uWL, 'All');
    C2{i} = coeffs(ydotdot(i), uWR, 'All');
    C3{i} = coeffs(ydotdot(i), udeltaL, 'All');
    C4{i} = coeffs(ydotdot(i), udeltaR, 'All');
    
    C1{i}(end) = [];
    C2{i}(end) = [];
    C3{i}(end) = [];
    C4{i}(end) = [];
    
    %simplify(ydotdot(i) - C1{i}*uWL - C2{i}*uWR - C3{i}*udeltaL - C4{i}*udeltaR)
    % sym - Empty sym: 1-by-0 = Empty sym: 1-by-0. So need to convert empty
    % arrays to zeros first.
    
end
C = {C1,C2,C3,C4};
%%
% Now go through each element: C1{:}, C2{:}, etc.
for j=1:numel(C)
    for i=1:numel(ydot)
        if isempty(C{j}{i})
            Cm(i,j) = 0;
        else
            Cm(i,j) = C{j}{i};
        end
    end
end

C1m = Cm(:, 1);
C2m = Cm(:, 2);
C3m = Cm(:, 3);
C4m = Cm(:, 4);
%Cm = [C1m, C2m, C3m, C4m];

%% check completeness
% idea is to remove all the input dependent terms from ydotdot and only
% leave constant terms behind.

alpha = simplify(ydotdot-C1m*uWL - C2m*uWR - C3m*udeltaL - C4m*udeltaR);
% One could find alpha by evaluating ydotdot with all inputs = 0 as well.
beta = Cm;
u = [uWL, uWR, udeltaL, udeltaR]';

% Check sanity
simplify(ydotdot - (alpha + beta*u)); % which is equal to 0 for all rows, good!

simplify(beta*inv(beta)); % Is identity matrix?
simplify(inv(beta)); % Check for div by 0? Only occur if an engine stop rotating.

%% Control law

% beta * inv(alpha) != 0 => good.

xdot = statesdot;
h = y;
f = subs(xdot,u,[0,0,0,0]');
gn = subs(simplify(xdot-f),u,[1,1,1,1]');
clear g
g(11:14,1:4)=diag(gn(11:14));
simplify(xdot-(f+g*u) ); % check if the calucation is sorrect!

L_gh=jacobian(h,states)*g;
L_fh=jacobian(h,states)*f;
L_f_gh=jacobian(L_fh, states )*g; % M?
L_f_fh =jacobian(L_fh, states )*f; % alpha?

M = L_f_gh;
new_alpha = L_f_fh; % Which equals old alpha.

%E=[ L_f_gh(1)*L_f_fh(1), L_f_gh(2)*L_f_fh(1), L_f_gh(3)*L_f_fh(1), L_f_gh(4)*L_f_fh(1) ; ...
%    L_f_gh(1)*L_f_fh(2), L_f_gh(2)*L_f_fh(2), L_f_gh(3)*L_f_fh(2), L_f_gh(4)*L_f_fh(2) ; ...
%    L_f_gh(1)*L_f_fh(3), L_f_gh(2)*L_f_fh(3), L_f_gh(3)*L_f_fh(3), L_f_gh(4)*L_f_fh(3) ; ...
%    L_f_gh(1)*L_f_fh(4), L_f_gh(2)*L_f_fh(4), L_f_gh(3)*L_f_fh(4), L_f_gh(4)*L_f_fh(4) ];


syms uWL_ref uWR_ref udeltaL_ref uDeltaR_ref real
refs = [uWL_ref, uWR_ref, udeltaL_ref, uDeltaR_ref]';


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

Lwx = -[1,2];
Lwy = -[1,2];
Lwz = -[1,2];
Lvx = -[1,2];
L = [Lwx, 0, 0, 0, 0, 0, 0; ...
    0, 0, Lwy, 0, 0, 0, 0; ...
    0, 0, 0, 0, Lwz, 0, 0; ...
    0, 0, 0, 0, 0, 0, Lvx];
Lrwx=1;
Lrwy=1;
Lrwz=1;
Lrvx=1;

Lr = [Lrwx, Lrwy, Lrwz, Lrvx]' .* refs;

xi = [wx, wxdot, wy, wydot, wz, wzdot, vx, vxdot]';

u_ref = inv(M)*(-L*xi + Lr - alpha)

