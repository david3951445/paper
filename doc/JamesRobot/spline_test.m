%%% 確定 cubic spline 加速度會是連續的 %%%

clc;clear;

x = [0,0  ,0   ,0  ,0   ,0  ,0,0,0,0  ,0,0];
y = [0,2  ,3   ,3.5,4   ,5  ,7,5,4,3.5,3,2];
z = [0,0.1,0.15,0.175,0.15,0.1,0,0,0,0,0,0];

x1 = [x,x,x];
y1 = [y,y,y];
z1 = [z,z,z];

t = 0:length(x1)-1;
tt = t(1):0.1:t(end);
xx = interp1(t,x1,tt,'spline');
yy = interp1(t,y1,tt,'spline');
zz = interp1(t,z1,tt,'spline');

vy = [0,diff(yy)/0.1];
ay = [0,diff(vy)/0.1];
cy = [0,diff(ay)/0.1];
figure;
plot3(xx,yy,zz);
figure;
plot(yy,zz);
figure;
comet(yy,zz,0.1);
figure;
plot(tt,vy);
figure;
plot(tt,ay);
figure;
plot(tt,cy);
%% inter 
% Define the sample points, x, and corresponding sample values, v.

x = 0:pi/4:2*pi; 
v = sin(x);
% Define the query points to be a finer sampling over the range of x.

xq = 0:pi/16:2*pi;
%  the function at the query points and plot the result.

figure
vq1 = interp1(x,v,xq);
plot(x,v,'o',xq,vq1,':.');
xlim([0 2*pi]);
title('(Default) Linear Interpolation');
% Now evaluate v at the same points using the 'spline' method.

figure
vq2 = interp1(x,v,xq,'spline');
plot(x,v,'o',xq,vq2,':.');
xlim([0 2*pi]);
title('Spline Interpolation');

%% quintic polynomial interpolation 
% Use the quinticpolytraj function with a given set of 2-D xy waypoints. Time points for the waypoints are also given.
wpts = [1 4 4 3 -2 0; 0 1 2 4 3 1];
tpts = 0:5;
% Specify a time vector for sampling the trajectory. Sample at a smaller interval than the specified time points.

tvec = 0:0.01:5;
% Compute the quintic trajectory. The function outputs the trajectory positions (q), velocity (qd), acceleration (qdd), and polynomial coefficients (pp) of the quintic polynomial.

[q, qd, qdd, pp] = quinticpolytraj(wpts, tpts, tvec);
% Plot the quintic trajectories for the x- and y-positions. Compare the trjactory with each waypoint.

plot(tvec, q)
hold all
plot(tpts, wpts, 'x')
xlabel('t')
ylabel('Positions')
legend('X-positions','Y-positions')
hold off