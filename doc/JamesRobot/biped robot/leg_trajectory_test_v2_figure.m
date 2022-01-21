clc;clear;
load leg_trajectory_test_v2.mat

close all
%% compare speed 
figure;
subplot(211)
plot(t,x);
xlabel('time (s)');
ylabel('x-axis coordinate (m)');
title('swing foot trajectory of a single step on x-axis x(t)');
grid on
subplot(212)
plot(t1,x1);
xlabel('time (s)');
ylabel('x-axis coordinate (m)');
title('swing foot trajectory of a single step on x-axis x(t)');
grid on

figure;
subplot(211)
plot(t,z);
xlabel('time (s)');
ylabel('z-axis coordinate (m)');
title('swing foot trajectory of a single step on z-axis z(t)');
grid on
subplot(212)
plot(t1,z1);
xlabel('time (s)');
ylabel('z-axis coordinate (m)');
title('swing foot trajectory of a single step on z-axis z(t)');
grid on

figure;
subplot(211)
plot(x,z);
xlabel('x-axis coordinate (m)');
ylabel('z-axis coordinate (m)');
title('swing foot trajectory of a single step x-z');
grid on
subplot(212)
plot(x1,z1);
xlabel('x-axis coordinate (m)');
ylabel('z-axis coordinate (m)');
title('swing foot trajectory of a single step x-z');
grid on

figure;
subplot(211)
plot(t,vx);
xlabel('time (s)');
ylabel('x-axis velocity (m/s)');
title('swing foot velocity of a single step on x-axis vx(t)');
grid on
subplot(212)
plot(t1,vx1);
xlabel('time (s)');
ylabel('x-axis velocity (m/s)');
title('swing foot velocity of a single step on x-axis vx(t)');
grid on

%% test cubic spline
figure;
plot(tq,zq);
xlabel('time (s)');
ylabel('z-axis coordinate (m)');
title('swing foot trajectory of a single step on z-axis z(t)');
grid on

figure;
plot(tq,xq);
xlabel('time (s)');
ylabel('x-axis coordinate (m)');
title('swing foot trajectory of a single step on x-axis x(t)');
grid on

figure;
plot(xq,zq);
xlabel('x-axis coordinate (m)');
ylabel('z-axis coordinate (m)');
title('swing foot trajectory of a single step x-z');
grid on

figure;
plot(tq_v,vxq);
xlabel('time (s)');
ylabel('x-axis velocity (m/s)');
title('swing foot velocity of a single step on x-axis vx(t)');
grid on

figure;
plot(tq_v,vzq);
xlabel('time (s)');
ylabel('z-axis velocity (m/s)');
title('swing foot velocity of a single step on z-axis vz(t)');
grid on

figure;
plot(tq_a,axq);
xlabel('time (s)');
ylabel('x-axis acceleration (m/s^2)');
title('swing foot acceleration of a single step on x-axis ax(t)');
grid on

figure;
plot(tq_a,azq);
xlabel('time (s)');
ylabel('z-axis acceleration (m/s^2)');
title('swing foot acceleration of a single step on z-axis az(t)');
grid on

%% the effect of change of vx to gait

figure;
plot(tqc,vxcq);
xlabel('time (s)');
ylabel('x-axis velocity (m/s)');
title('swing foot velocity of a single step on x-axis vx(t)');
grid on

figure;
plot(tqc,xc);
xlabel('time (s)');
ylabel('x-axis coordinate (m)');
title('swing foot trajectory of a single step on x-axis x(t)');
grid on

figure;
plot(tqc,Tcq);
xlabel('time (s)');
ylabel('Ts');
title('Ts');
grid on

figure;
plot(tqc,S./Tcq);
xlabel('time (s)');
ylabel('S/Tcq');
title('S/Tcq');
grid on