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

syms dA jA real;
inputs = [dA, jA];
references = [tx, ty];
deltaL = jA - dA/2;
deltaR = jA + dA/2;

% We want to solve for the inputs given the state and references.

eq1 = tx == p.yf.*(FLT+FL -FRT-FR);
eq1 = subs(eq1);
eq2 = ty == p.xf.*(FLT+FL+FRT+FR) -p.xac.*(LL+LR+LLT+LRT);
eq2 = subs(eq2);

sol = solve(eq1,eq2,dA,jA);


