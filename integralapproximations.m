%params;
c = @(y) 0.28 - (0.28-0.09).*abs(y)./(p.b/2); % This should be pretty accurate

vxs = -3:0.5:20;
vzs = -5:0.5:5;
vys = -2:0.1:2;
Nvx = numel(vxs);
Nvz = numel(vzs);
wxs = -1:0.05:1;
wzs = -1:0.05:1;
f1 = zeros(numel(wxs),numel(vzs),numel(vxs)); % flipped!
f2 = f1;
f3 = f1;
f31l = f1;
f2l = f1;
f3l = f1;
CL = @(alpha) 1.2*sin(2.*alpha)+0.2; % These are wrong, just placeholders
CD = @(alpha) 2-2.*cos(2.*alpha)+0.05;

%% Restmomroll
parfor k = 1:numel(wxs)
    for i = 1:Nvx
        for j = 1:Nvz
            vx = vxs(i);
            vz = vzs(j);
            wx = wxs(k);
            alphal_rmr = @(y) atan2(vz+wx.*y, vx);
            restmomroll = @(y) y .* c(y) .* sqrt(vx.^2 + (vz+wx.*y).^2) .* ...
                (CL(alphal_rmr(y)).*vx + CD(alphal_rmr(y)).*(vz+wx.*y));
            f1(k,j,i) = -0.5*p.rho.*quadgk(restmomroll,-p.yW,p.yW) ...     % between wakes
                       -0.5*p.rho.*quadgk(restmomroll,-p.yWEE,-p.yWE) ... % outside left wake
                       -0.5*p.rho.*quadgk(restmomroll,p.yWE,p.yWEE);      % outside right wake
        end
    end
end

%%
figure(1);
hold on;
s1 = surf(vxs, vzs, squeeze(f1(1,:,:)));
xlabel('vx');
ylabel('vz');
zlabel('restmom roll');
view([1,1,1]);
zmin = min(f1,[],'all');
zmax = max(f1,[],'all');
axis([-inf,inf,-inf,inf,zmin,zmax]);
colorbar;
caxis([zmin,zmax]);
%shading interp;
axis equal;
hold off;

fwd = 1;
k = 1;
N = numel(wxs);
while 1
   s1.ZData = squeeze(f1(k,:,:));
   axis([-inf,inf,-inf,inf,zmin,zmax]);
   caxis([zmin,zmax]);
   title('restoring moment due to roll, omegax: '+string(wxs(k)));
   if k==N
       fwd=0;
   end
   if k==1
       fwd=1;
   end
   if fwd
       k=k+1;
   else
       k=k-1;
   end
   pause(0.05);
    
end

%% Data to ue for linfits
ZRMR = squeeze(f1(end,:,:));
% polys of 3x2 or 5x5 create good results.

%% Rollmomyaw
parfor k = 1:numel(wzs)
    for i = 1:Nvx
        for j = 1:Nvz
            vx = vxs(i);
            vz = vzs(j);
            wz = wzs(k);
            alphal_rwz = @(y) atan2(vz, vx-wz.*y);
            rollduetowz = @(y) y .* c(y) .* sqrt((vx-wz.*y).^2 + vz.^2) .* ...
                (CL(alphal_rwz(y)).*(vx-wz*y) + CD(alphal_rwz(y)).*vz);
            f2(k,j,i) = -0.5*p.rho.*quadgk(rollduetowz,-p.yW,p.yW) ... between wakes
                         -0.5*p.rho.*quadgk(rollduetowz,-p.yWEE,-p.yWE) ... left side
                         -0.5*p.rho.*quadgk(rollduetowz, p.yWE,p.yWEE);  % right side
        end
    end
end

%%
figure(2);
hold on;
s2 = surf(vxs, vzs, squeeze(f2(1,:,:)));
xlabel('vx');
ylabel('vz');
zlabel('restmom yawrate');
view([1,1,1]);
zmin = min(f2,[],'all');
zmax = max(f2,[],'all');
axis([-inf,inf,-inf,inf,zmin,zmax]);
colorbar;
caxis([zmin,zmax]);
%shading interp;
axis equal;
hold off;

fwd = 1;
k = 1;
N = numel(wzs);
while 1
   s2.ZData = squeeze(f2(k,:,:));
   axis([-inf,inf,-inf,inf,zmin,zmax]);
   caxis([zmin,zmax]);
   title('the rolling moment due to yawrate, omegaz: '+string(wzs(k)));
   if k==N
       fwd=0;
   end
   if k==1
       fwd=1;
   end
   if fwd
       k=k+1;
   else
       k=k-1;
   end
   pause(0.05);
    
end
%%
ZRollMY = squeeze(f2(end,:,:));
% 2 by 3 is good

%% Restmomyaw
parfor k = 1:numel(wzs)
    for i = 1:Nvx
        for j = 1:Nvz
            vx = vxs(i);
            vz = vzs(j);
            wz = wzs(k);
            alphal_rwz = @(y) atan2(vz, vx-wz.*y);
            restmomwz = @(y) y .* c(y) .* sqrt((vx-wz.*y).^2 + vz.^2) .* ...
                (-CL(alphal_rwz(y)).*vz + CD(alphal_rwz(y)).*(vx-wz.*y));
            f3(k,j,i) = 0.5*quadgk(restmomwz,-p.yW,p.yW) ...
                         +0.5*quadgk(restmomwz,-p.yWEE,-p.yWE) ...
                         +0.5*quadgk(restmomwz, p.yWE,p.yWEE);
        end
    end
end

%%
figure(3);
hold on;
s3 = surf(vxs, vzs, squeeze(f3(1,:,:)));
xlabel('vx');
ylabel('vz');
zlabel('restmom yawrate');
view([1,1,1]);
zmin = min(f3,[],'all');
zmax = max(f3,[],'all');
axis([-inf,inf,-inf,inf,zmin,zmax]);
colorbar;
caxis([zmin,zmax]);
%shading interp;
axis equal;
hold off;

fwd = 1;
k = 1;
N = numel(wzs);
while 1
   s3.ZData = 10.*squeeze(f3(k,:,:));
   axis(10.*[-inf,inf,-inf,inf,zmin,zmax]);
   caxis(10.*[zmin,zmax]);
   title('10 times the restoring moment due to yawrate, omegaz: '+string(wzs(k)));
   if k==N
       fwd=0;
   end
   if k==1
       fwd=1;
   end
   if fwd
       k=k+1;
   else
       k=k-1;
   end
   pause(0.05);
    
end

%% 
ZRMY = squeeze(f3(end,:,:));
% 3 by 3 is perhaps good enough.
