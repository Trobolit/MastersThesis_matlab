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


% We assume propellers always spin one way. Never reverse.
% We know wanted torque, i.e. we know wdot*Iz=taoz=p.yt*(TL-TR)=p.yt*dT
% taoz./p.yt = TL-TR = p.K_T*(omegaL^2-omegaR^2)
% taoz/(p.yt*p.K_T) = omegaL^2-omegaR^2
% we also know
%TL = JT+dT/2
%TR = JT-dt/2
% => TL-TR = dT = taoz
% if TL=p.K_T*omegaL^2 then
%omegaL=sqrt(TL/p.K_T) =>
%omegaL=sqrt( (JT+dt/2)/p.K_T )
u_omegaL2 = (u_jointT+(tz/p.yt)/2)/p.K_T ; % THese share the jointT, half each.
u_omegaR2 =  (u_jointT-(tz/p.yt)/2)/p.K_T ;

u_omegaL = sqrt(max(u_omegaL2,0)); % This enforces no backwards spinning
u_omegaR = sqrt(max(u_omegaR2,0));


% Lets make a few (crude) approximations.
% - Damping due to rotational speed doesn't exist.
% - Winglets don't exist
% Then we come to these formulas for torque around each axis:
%wxdot*Ix = p.yf.*(FLT+FL -FRT-FR)
%wydot*Iy = p.xf.*(FLT+FL+FRT+FR) -p.xac.*(LL+LR+LLT+LRT) 
%wzdot*Iz = p.yt.*(TL-TR) ...
% 
% They essentially say that aileroins are effective proportioanlly to the
% (square) of the air velocity over them.
% We already solved for f3, but the others are coupled, lets decouple.

inputs = [deltaL, deltaR];
references = [tx, ty];

% First calculate joint deflection for each wing sperataley.

%left wing, pitch moment:
eq_Ly = ty/2 == p.xf.*(FLT+FL) -p.xac.*(LL+LLT);
deltaL_y = solve(eq_Ly,deltaL);
%right wing, pitch moment:
eq_Ry = ty/2 == p.xf.*(FRT+FR) -p.xac.*(LR+LRT);
deltaR_y = solve(eq_Ry,deltaR);

% Now calculate extra deflection to produce roll moment on each wing.
% Remember that sensitivity of the actuator may depend on joint setting.
syms deltaL_x deltaR_x real;
eq_Lx = tx/2 == p.yf.*(FLT+FL);
%eq_Lx = subs(eq_Lx,deltaL,deltaL_y+deltaL_x);
deltaL_x = solve(eq_Lx,deltaL);

eq_Rx = tx/2 == p.yf.*(-FRT-FR);
%eq_Rx = subs(eq_Rx,deltaR,deltaR_y+deltaR_x);
deltaR_x = solve(eq_Rx,deltaR);

deltaL = deltaL_x + deltaL_y
deltaR = deltaR_x + deltaR_y

vpa(deltaL,3)
vpa(deltaR,3)

% DONT SIMPLIFY
%deltaL = simplify(deltaL);
%deltaR = simplify(deltaR);
% Even though the expression becomes shorter, the numerical insertions we
% have from the parameters file (I believe) makes exponents go to 10^50
% range...