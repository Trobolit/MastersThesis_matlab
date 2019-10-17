
tao = [1,1];
hoverpower = 0.64.*[1,1];
cruiseVel = 15;
cruiseThrottle = 0.5;
axisgains = 3.7*[1,1,nan]; 

k2 = 0.5/(axisgains(2)*hoverpower(1).^2);
k4 = 0.5/(axisgains(1)*hoverpower(1).^2);

k1 = (0.5/axisgains(2) - k2*cruiseThrottle^2)/(cruiseVel^2);
k3 = (0.5/axisgains(1) - k4*cruiseThrottle^2)/(cruiseVel^2);

%u_deltaL = -0.5*tao(2)   /    (k1*sign(v(1))*sum(v.^2) + k2*omegaL^2) ...
%           - 0.5*tao(1)/(k3*sign(v(1))*sum(v.^2) + k4*omegaL^2);

%u_deltaR = -0.5*tao(2)/(k1*sign(v(1))*sum(v.^2) + k2*omegaR^2) ...
%           + 0.5*tao(1)/(k3*sign(v(1))*sum(v.^2) + k4*omegaR^2);
       
       
% At hover power is roughly 0.28, or 0.64 if [0,1].
% Velocity 0, and torques are passed through as is, so if
% v=0, TL=0.64 and we have P=0.6, D=0.1 and C=[3.4,3.4,n/a]
% 
% Manually we found that tao*3.7 at hover was nice, for both x and y.
% 0.5 /( 0 + k2*omegaR^2) = 3.7/taoy =>
% 0.5/(k2*0.64^2) = 3.7/ taoy =>
% k2 = 0.33
% We also know that at flight a constant of 1 worked (albeit with much
% lower D gain)
% Lets assume we flew at roughly 15 m/s, and at maybe half throttle? (Need
% to verify that with tests!)
% 0.5/(k1*v^2 + 0.33*0.5^2) = 3.7 =>
% k1 = (0.5/3.7 - 0.33*0.5^2)/(15^2) = 0.000234 