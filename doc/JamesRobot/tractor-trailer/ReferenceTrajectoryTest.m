clc;clear;
h = 0.001;
t = [0;1;2;4;5];
x1 = [1;3;4;5;7];
y1 = [2;4;3;6;7];
tt = 0:h:5;
l = length(tt);
d = 0.5;
%%%%%%%%%% by holonomic comstraints %%%%%%%%%
%%% linear velocity %%% 
xx1 = spline(t,x1,tt);
yy1 = spline(t,y1,tt);
vx1 = diff(xx1)/h;
vy1 = diff(yy1)/h;
ax1 = diff(vx1)/h;
ay1 = diff(vy1)/h;
jx1 = diff(ax1)/h;
jy1 = diff(ay1)/h;
%%% angular velocity %%%
th1 = atan2d(vy1,vx1);
w1 = diff(th1)/h;
afa1 = diff(w1)/h;

%%% linear velocity %%%
xx0 = xx1(1:l-1) + d*cosd(th1);
yy0 = yy1(1:l-1) + d*sind(th1);
vx0 = diff(xx0)/h;
vy0 = diff(yy0)/h;
ax0 = diff(vx0)/h;
ay0 = diff(vy0)/h;
jx0 = diff(ax0)/h;
jy0 = diff(ay0)/h;
%%% angular velocity %%%
th0 = atan2d(vy0,vx0);
w0 = diff(th0)/h;
afa0 = diff(w0)/h;

%%%%%%%%%% by nonholonomic comstraints %%%%%%%%%
len = 4001; % deal with the problem of matrix dimension
%%% linear velocity %%%
v = (vx0(1:len).^2+vy0(1:len).^2).^(1/2);
vx2 = v.*cosd(th1(1:len)).*cosd(th0(1:len)-th1(1:len));
vy2 = v.*sind(th1(1:len)).*cosd(th0(1:len)-th1(1:len));
w2 = v.*sind(th0(1:len)-th1(1:len))/d*180/pi; %% rad to degree


save ReferenceTrajectoryTest.mat