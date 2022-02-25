clc;clear;close all;
% A = [1,1];
% B = [3,2];
% C = [4.5,1];
% D = [6.3,2];
% E = [5.2,3.5];
% F = [8,5];

x = [1 3 4.5 6.3 5.2 8]
y = [1 2 1 2 3.5 5]
t = [0 20 40 60 80 100]*5
% px = polyfit(t,x,5);
% py = polyfit(t,y,5);
h = 1e-3;
dt = h;
t1 = 0:h:100*5;

x1 = interp1(t,x,t1,'spline');
y1 = interp1(t,y,t1,'spline');

% x1 = polyval(px,t1);
% y1 = polyval(py,t1);

figure
plot(t,x,'o')
hold on
plot(t1,x1)

figure
plot(t,y,'o')
hold on
plot(t1,y1)

figure
plot(x,y,'o')
hold on
plot(x1,y1)



vx = diff(x1)/dt;
vy = diff(y1)/dt;
ax = diff(vx)/dt;
ay = diff(vy)/dt;
figure;
plot(t1(1:end-1),vx);
figure;
plot(t1(1:end-1),vy);
figure;
plot(t1(1:end-2),ax);
figure;
plot(t1(1:end-2),ay);



