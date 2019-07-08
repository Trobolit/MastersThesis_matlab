clear all;

syms q0 q1 q2 q3 v1 v2 v3 mg wx wy wz real

q = [q0,q1,q2,q3];
v = [0,v1,v2,v3];
w = [wx, wy, wz];

quatmultiply(q,quatmultiply([0,0,0,mg],quatconj(q)))';

-0.5*quatmultiply([0,w],q)'

%% Try rotating unit vector v around w
v = [1,1,0]/sqrt(2);
w = [1,0,0];
d = -90; % degrees, -framerot
q = [cosd(d/2),sind(d/2).*w];

quatmultiply(q,quatmultiply([0,v],quatconj(q)))'

