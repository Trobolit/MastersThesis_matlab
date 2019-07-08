wantedpoints = 200;
numpoints = numel(q_wf.Time);
N = numpoints;
%skip=numpoints/wantedpoints;

%Generate vector of wapoints for lines to get
wpline = p.waypoints(:,1:3);

skip=3;
e = N;
start = 1; %N-10*skip;
scaleg = 0;
v = a_bf.Data(start:skip:e,:);
ref = rr.Data(start:skip:e,:);
gs = gravity_bf.Data(start:skip:e,:);
gs = gs.*scaleg;
refvecs = 100.*refvec.Data(start:skip:e,:);
scalea = 50;
xaxis = scalea.*[1,0,0];
yaxis = scalea.*[0,1,0]; 
zaxis = scalea.*[0,0,1]; 
scalev = 0.25;%0.25;
vx = v(:,1).*scalev.*[1,0,0];
vy = v(:,2).*scalev.*[0,1,0]; 
vz = v(:,3).*scalev.*[0,0,1];


qs = q_wf.Data(start:skip:e,:);
xaxises = quatrotate(qs,xaxis);
yaxises = quatrotate(qs,yaxis);
zaxises = quatrotate(qs,zaxis);
vxs = quatrotate(qs,vx);
vys = quatrotate(qs,vy);
vzs = quatrotate(qs,vz);
rs = r.Data(start:skip:e,:);
g = quatrotate(qs,gs);

f = figure(69);
hold on;
quiver3(rs(:,1),rs(:,2),rs(:,3),xaxises(:,1),xaxises(:,2),xaxises(:,3),0,'x','color',[0.4 0 0]);
quiver3(rs(:,1),rs(:,2),rs(:,3),yaxises(:,1),yaxises(:,2),yaxises(:,3),0,'o','color',[0 0.4 0]);
quiver3(rs(:,1),rs(:,2),rs(:,3),zaxises(:,1),zaxises(:,2),zaxises(:,3),0,'o','color',[0 0 0.4]);
quiver3(rs(:,1),rs(:,2),rs(:,3),vxs(:,1),vxs(:,2),vxs(:,3),0,'color',[1 0.5 0]);
quiver3(rs(:,1),rs(:,2),rs(:,3),vys(:,1),vys(:,2),vys(:,3),0,'color',[1 0 1]);
quiver3(rs(:,1),rs(:,2),rs(:,3),vzs(:,1),vzs(:,2),vzs(:,3),0,'color',[0 0 0]);
quiver3(rs(:,1),rs(:,2),rs(:,3),g(:,1),g(:,2),g(:,3),0,'color',[1 0 0]);
%scatter3(ref(:,1),ref(:,2),ref(:,3),'x');
quiver3(rs(:,1),rs(:,2),rs(:,3),refvecs(:,1),refvecs(:,2),refvecs(:,3),0,'color',[1 0 0]);
scatter3(p.waypoints(:,1),p.waypoints(:,2),p.waypoints(:,3),0.001*pi*p.wpradius.^2);
line(wpline(:,1),wpline(:,2),wpline(:,3));
xlabel('x');
ylabel('y');
zlabel('z');
legend('x_bf','y_bf','z_bf','ax','ay','az','g','reference velocity integrated');
set(gca, 'Zdir','reverse'); % To get plot in right shape for NED
set(gca, 'ydir','reverse');
set(gca, 'Clipping','off');
view([-1,1,1])
axis equal
%set(gca,'color','k') %background color
set(gca,'CameraViewAngleMode','Manual')
hold off