clear all
syms theta Psy phi mg real

% Book: 1.yaw(z,Psy), 2.pitch(y, theta), 3.roll(x, phi)

rotPsy = [cos(Psy), -sin(Psy), 0; sin(Psy), cos(Psy), 0; 0,0,1]; %z
rotTheta = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)]; %y
rotPhi = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)]; %x

rotmg = rotPhi'*rotTheta'*rotPsy';
Fz = rotmg*[0;0;mg]

% Looking for [-mg St ; mg CtSp ; mg Ct Cp ], check! The Psy also does not
% contribute as expected, nice.