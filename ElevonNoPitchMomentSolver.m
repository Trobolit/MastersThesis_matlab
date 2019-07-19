clear all;
params;

% The reference coming in is for angular accelleration, so don't treat it
% as pure torque:
% I*wdot = tao 

syms tx ty tz  vx vy vz omegaL omegaR deltaL deltaR u_jointT real;

v_w_L = p.b_w .* omegaL; %[b_w .* omegaL; vy; vz];
v_w_R = p.b_w .* omegaR;
% AILERONS
% Lift inside wake, signs bc rotation dir of aileron
FLT = -p.CLdelta * 0.5*p.rho * p.Saw * deltaL*v_w_L.^2;
FRT = -p.CLdelta * 0.5*p.rho * p.Saw * deltaR*v_w_R.^2;
% Lift outside wake
FL = -sign(vx).*p.CLdelta * 0.5*p.rho*(vx.^2 + vy.^2 + vz.^2) * p.Sa* deltaL;
FR = -sign(vx).*p.CLdelta * 0.5*p.rho*(vx.^2 + vy.^2 + vz.^2) * p.Sa* deltaR;
% function handles not supported, so adding here directly
alpha = atan2(vz,vx);
c = @(y) 0.28 - (0.28-0.09).*abs(y)./(p.b/2); % This should be pretty accurate
% Lift curves
CL = @(alpha) p.CL_A*sin(p.CL_B.*alpha)+p.CL_C; % These are wrong, just placeholders
CD = @(alpha) p.CD_A-p.CD_A.*cos(p.CD_B.*alpha)+p.CD_C;
% MAIN WING
% Lift inside wake
LLT = p.Sw_w * 0.5 * p.rho * abs(v_w_L) .* CL(0).*v_w_L ;
LRT = p.Sw_w * 0.5 * p.rho * abs(v_w_R) .* CL(0).*v_w_R ;
% Lift outside wake
LL = p.Sw * 0.5 * p.rho * sqrt(vx.^2 + vz.^2) .* ...
    (CL(alpha).*vx + CD(alpha).*vz);
LR = LL; % Yeah we -could- combine these, but too late!

% Solve for no pitch moment
eq_Ly = 0 == p.xf.*(FLT+FL) -p.xac.*(LL+LLT);
deltaL_y = solve(eq_Ly,deltaL);

%vx = -2:0.1:20;
%vz = -2:0.1:2;




