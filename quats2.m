syms q00 q01 q02 q03 q10 q11 q12 q13 q20 q21 q22 q23 real;

%%
q_init = [cos(-pi/4), 0, sin(-pi/4), 0];
qyaw = [q00 0 0 q03];
qpitch = [q10 0 q12 0];
qroll = [q20 q21 0 0];

q0 = quatmultiply(q_init,qyaw)
q1 = quatmultiply(q0, qpitch);
q = quatmultiply(q1,qroll)'