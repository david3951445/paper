clc;clear;
load leg_trajectory_test_v3.mat

close all
%% compare speed 
figure;
plot(t,x);
xlabel('time (s)');
ylabel('x-axis coordinate (m)');
title('swing foot trajectory of a single step on x-axis x(t)');
grid on

figure;
plot(t,z);
xlabel('time (s)');
ylabel('z-axis coordinate (m)');
title('swing foot trajectory of a single step on z-axis z(t)');
grid on

figure;
plot(x,z);
xlabel('x-axis coordinate (m)');
ylabel('z-axis coordinate (m)');
title('swing foot trajectory of a single step x-z');
grid on

figure;
plot(t,vx);
xlabel('time (s)');
ylabel('x-axis velocity (m/s)');
title('swing foot velocity of a single step on x-axis vx(t)');
grid on
%% test cubic spline
figure;
plot(tq,zq);
xlabel('time (s)');
ylabel('z-axis coordinate (m)');
title('swing foot trajectory in a single step on z-axis z(t)');
grid on

figure;
plot(tq,xq);
xlabel('time (s)');
ylabel('x-axis coordinate (m)');
title('swing foot trajectory in a single step on x-axis x(t)');
grid on

figure;
plot(xq,zq);
xlabel('x-axis coordinate (m)');
ylabel('z-axis coordinate (m)');
title('swing foot trajectory of a single step x-z (cubic spline)');
grid on

figure;
plot(tq_v,vxq);
xlabel('time (s)');
ylabel('x-axis velocity (m/s)');
title('swing foot velocity in a single step on x-axis vx(t)');
grid on

figure;
plot(tq_v,vzq);
xlabel('time (s)');
ylabel('z-axis velocity (m/s)');
title('swing foot velocity in a single step on z-axis vz(t)');
grid on

figure;
plot(tq_a,axq);
xlabel('time (s)');
ylabel('x-axis acceleration (m/s^2)');
title('swing foot acceleration in a single step on x-axis axq(t)');
grid on

figure;
plot(tq_a,azq);
xlabel('time (s)');
ylabel('z-axis acceleration (m/s^2)');
title('swing foot acceleration in a single step on z-axis azq(t)');
grid on