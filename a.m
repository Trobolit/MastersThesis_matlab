clear all
syms wx wy wz Ix Iy Iz vx vy vz m real

cross([wx,wy,wz], [vx,vy,vz])

I = eye(3).*[Ix;Iy;Iz];
w = [wx,wy,wz];

simplify(cross(w, I*w')')