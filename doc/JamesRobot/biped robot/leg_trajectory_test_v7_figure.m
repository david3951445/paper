clc;clear; close all
load leg_trajectory_test_v7.mat

% figure;
% comet(gait(1,:),gait(4,:));
% title('x-z');xlabel('x(m)');ylabel('z(m)');
% axis equal;
% grid on;

figure;
plot(gait(1,:),gait(4,:));
title('x-z');xlabel('x(m)');ylabel('z(m)');
axis equal;
grid on;

figure;
plot(tr,gait(1,:));
title('t-x');xlabel('t(s)');ylabel('x(m)');
grid on;

figure;
plot(tr,gait(2,:));
title('t-vx');xlabel('t(s)');ylabel('vx(m/s)');
grid on;

figure;
plot(tr,gait(3,:));
title('t-ax');xlabel('t(s)');ylabel('ax(m/s^2)');
grid on;

figure;
plot(tr,gait(4,:));
title('t-z');xlabel('t(s)');ylabel('z(m)');
grid on;

figure;
plot(tr,gait(5,:));
title('t-vz');xlabel('t(s)');ylabel('vz(m/s)');
grid on;

figure;
plot(tr,gait(6,:));
title('t-az');xlabel('t(s)');ylabel('az(m/s^2)');
grid on;

figure;
plot(v,gait(2,:),'o');
title('v-vx');
grid on;
