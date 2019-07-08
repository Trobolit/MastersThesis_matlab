clear vxs;

ds = 0:1:90;
N = numel(ds);
vxs = zeros(N,1);

for i=1:N
    d = (90-ds(i))/2;
    d
    sim TailSitter_waypoints;
    vxs(i) = mean(v_wf.Data(500:end,1));
end

plot(ds,vxs);

%{
    f(x) = p1*x + p2
Coefficients (with 95% confidence bounds):
       p1 =      0.2685  (0.2581, 0.2788)
       p2 =      -6.013  (-6.552, -5.474)
       If angle in radians:
       p1 =       15.38  (14.79, 15.98)
       p2 =      -6.013  (-6.552, -5.474)
}
% Using 4th degree yeilds 0.99x R^2 value. but 1st order is good enough.

% vx_wf = p1*ds + p2
% vx_wf = p1*rads + p2

% (vx_wf - p2)/p1 = rads