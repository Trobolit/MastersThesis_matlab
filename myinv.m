function [invM] = myinv(M)
%MYINV Summary of this function goes here
%   Detailed explanation goes here

invM = M;

invM(1,1) = (M(2,2)*M(3,3) - M(2,3)*M(3,2))/(M(1,1)*M(2,2)*M(3,3) - M(1,1)*M(2,3)*M(3,2) - M(1,2)*M(2,1)*M(3,3) + M(1,2)*M(2,3)*M(3,1) + M(1,3)*M(2,1)*M(3,2) - M(1,3)*M(2,2)*M(3,1));
invM(1,2) = -(M(1,2)*M(3,3) - M(1,3)*M(3,2))/(M(1,1)*M(2,2)*M(3,3) - M(1,1)*M(2,3)*M(3,2) - M(1,2)*M(2,1)*M(3,3) + M(1,2)*M(2,3)*M(3,1) + M(1,3)*M(2,1)*M(3,2) - M(1,3)*M(2,2)*M(3,1));
invM(1,3) = (M(1,2)*M(2,3) - M(1,3)*M(2,2))/(M(1,1)*M(2,2)*M(3,3) - M(1,1)*M(2,3)*M(3,2) - M(1,2)*M(2,1)*M(3,3) + M(1,2)*M(2,3)*M(3,1) + M(1,3)*M(2,1)*M(3,2) - M(1,3)*M(2,2)*M(3,1));

invM(2,1) = -(M(2,1)*M(3,3) - M(2,3)*M(3,1))/(M(1,1)*M(2,2)*M(3,3) - M(1,1)*M(2,3)*M(3,2) - M(1,2)*M(2,1)*M(3,3) + M(1,2)*M(2,3)*M(3,1) + M(1,3)*M(2,1)*M(3,2) - M(1,3)*M(2,2)*M(3,1));
invM(2,2) = (M(1,1)*M(3,3) - M(1,3)*M(3,1))/(M(1,1)*M(2,2)*M(3,3) - M(1,1)*M(2,3)*M(3,2) - M(1,2)*M(2,1)*M(3,3) + M(1,2)*M(2,3)*M(3,1) + M(1,3)*M(2,1)*M(3,2) - M(1,3)*M(2,2)*M(3,1));
invM(2,3) = -(M(1,1)*M(2,3) - M(1,3)*M(2,1))/(M(1,1)*M(2,2)*M(3,3) - M(1,1)*M(2,3)*M(3,2) - M(1,2)*M(2,1)*M(3,3) + M(1,2)*M(2,3)*M(3,1) + M(1,3)*M(2,1)*M(3,2) - M(1,3)*M(2,2)*M(3,1));

invM(3,1) = (M(2,1)*M(3,2) - M(2,2)*M(3,1))/(M(1,1)*M(2,2)*M(3,3) - M(1,1)*M(2,3)*M(3,2) - M(1,2)*M(2,1)*M(3,3) + M(1,2)*M(2,3)*M(3,1) + M(1,3)*M(2,1)*M(3,2) - M(1,3)*M(2,2)*M(3,1));
invM(3,2) = -(M(1,1)*M(3,2) - M(1,2)*M(3,1))/(M(1,1)*M(2,2)*M(3,3) - M(1,1)*M(2,3)*M(3,2) - M(1,2)*M(2,1)*M(3,3) + M(1,2)*M(2,3)*M(3,1) + M(1,3)*M(2,1)*M(3,2) - M(1,3)*M(2,2)*M(3,1));
invM(3,3) = (M(1,1)*M(2,2) - M(1,2)*M(2,1))/(M(1,1)*M(2,2)*M(3,3) - M(1,1)*M(2,3)*M(3,2) - M(1,2)*M(2,1)*M(3,3) + M(1,2)*M(2,3)*M(3,1) + M(1,3)*M(2,1)*M(3,2) - M(1,3)*M(2,2)*M(3,1));


 


end

