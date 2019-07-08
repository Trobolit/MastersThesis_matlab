%%
%We assume x-z- being symmetry plane.
clear all
syms p q r Ix Iy Iz Ixz pdot qdot rdot real;
A = [Ix, 0, -Ixz; 0, Iy, 0; -Ixz, 0, Iz];
B = [q*r*(Iz-Iy)-Ixz*p*q; r*p*(Ix-Iz)+Ixz*(p^2-r^2); p*q*(Iy-Ix)+Ixz*q*r];
omegadot = [pdot;qdot;rdot];
M = A*omegadot + B;


omegadotsol = inv(A)*(M-B);

%% alt repr

J = A;
omega = [p;q;r];
-cross(omega,J*omega)
