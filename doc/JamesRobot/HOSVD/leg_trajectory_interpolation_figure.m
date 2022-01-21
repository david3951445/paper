clc;clear; close all
load 'a.mat'
%% 我們真正想表現在圖上的
% t = t(1:end-2);
% gait_f2 = gait_f2(:,1:end);
%% 
figure;
comet(gait_f_l1(1,:),gait_f_l1(4,:));
title('x-z(right)');xlabel('x(m)');ylabel('z(m)');
axis equal;
grid on;
%%
figure;
plot(gait_f2(7,:),gait_f2(10,:));
title('x-z(left)');xlabel('x(m)');ylabel('z(m)');
axis equal;
grid on;
%%
figure;
plot(t,gait_f2(1,:));
title('t-x(right)');xlabel('t(s)');ylabel('x(m)');
grid on;

figure;
plot(t,gait_f2(2,:));
title('t-vx(right)');xlabel('t(s)');ylabel('vx(m/s)');
grid on;

figure;
plot(t,gait_f2(3,:));
title('t-ax(right)');xlabel('t(s)');ylabel('ax(m/s^2)');
grid on;

figure;
plot(t,gait_f2(4,:));
title('t-z(right)');xlabel('t(s)');ylabel('z(m)');
grid on;

figure;
plot(t,gait_f2(5,:));
title('t-vz(right)');xlabel('t(s)');ylabel('vz(m/s)');
grid on;

figure;
plot(t,gait_f2(6,:));
title('t-az(right)');xlabel('t(s)');ylabel('az(m/s^2)');
grid on;
%%
figure;
plot(t,gait_f2(7,:));
title('t-x(left)');xlabel('t(s)');ylabel('x(m)');
grid on;

figure;
plot(t,gait_f2(8,:));
title('t-vx(left)');xlabel('t(s)');ylabel('vx(m/s)');
grid on;

figure;
plot(t,gait_f2(9,:));
title('t-ax(left)');xlabel('t(s)');ylabel('ax(m/s^2)');
grid on;

figure;
plot(t,gait_f2(10,:));
title('t-z(left)');xlabel('t(s)');ylabel('z(m)');
grid on;

figure;
plot(t,gait_f2(11,:));
title('t-vz(left)');xlabel('t(s)');ylabel('vz(m/s)');
grid on;

figure;
plot(t,gait_f2(12,:));
title('t-az(left)');xlabel('t(s)');ylabel('az(m/s^2)');
grid on;

figure;
plot(t,gait_f2(13,:));
title('t-q1');xlabel('t(s)');ylabel('q1(rad)');
grid on

figure;
plot(t,gait_f2(14,:));
title('t-q2');xlabel('t(s)');ylabel('q2(rad)');
grid on

figure;
plot(t,gait_f2(15,:));
title('t-q3');xlabel('t(s)');ylabel('q3(rad)');
grid on

figure;
plot(t,gait_f2(16,:));
title('t-q4');xlabel('t(s)');ylabel('q4(rad)');
grid on

figure;
plot(t,gait_f2(17,:));
title('t-q5');xlabel('t(s)');ylabel('q5(rad)');
grid on

figure;
plot(t,gait_f2(18,:));
title('t-q6');xlabel('t(s)');ylabel('q6(rad)');
grid on

figure;
plot(t,gait_f2(19,:));
title('t-q7');xlabel('t(s)');ylabel('q7(rad)');
grid on

figure;
plot(t,gait_f2(20,:));
title('t-q8');xlabel('t(s)');ylabel('q8(rad)');
grid on

figure;
plot(t,gait_f2(21,:));
title('t-dq1');xlabel('t(s)');ylabel('dq1(rad/s)');
grid on

figure;
plot(t,gait_f2(22,:));
title('t-dq2');xlabel('t(s)');ylabel('dq2(rad/s)');
grid on

figure;
plot(t,gait_f2(23,:));
title('t-dq3');xlabel('t(s)');ylabel('dq3(rad/s)');
grid on

figure;
plot(t,gait_f2(24,:));
title('t-dq4');xlabel('t(s)');ylabel('dq4(rad/s)');
grid on

figure;
plot(t,gait_f2(25,:));
title('t-dq5');xlabel('t(s)');ylabel('dq5(rad/s)');
grid on

figure;
plot(t,gait_f2(26,:));
title('t-dq6');xlabel('t(s)');ylabel('dq6(rad/s)');
grid on

figure;
plot(t,gait_f2(27,:));
title('t-dq7');xlabel('t(s)');ylabel('dq7(rad/s)');
grid on

figure;
plot(t,gait_f2(28,:));
title('t-dq8');xlabel('t(s)');ylabel('dq8(rad/s)');
grid on

figure;
plot(t,gait_f2(29,:));
title('t-ddq1');xlabel('t(s)');ylabel('ddq1(rad/s^2)');
grid on

figure;
plot(t,gait_f2(30,:));
title('t-ddq2');xlabel('t(s)');ylabel('ddq2(rad/s^2)');
grid on

figure;
plot(t,gait_f2(31,:));
title('t-ddq3');xlabel('t(s)');ylabel('ddq3(rad/s^2)');
grid on

figure;
plot(t,gait_f2(32,:));
title('t-ddq4');xlabel('t(s)');ylabel('ddq4(rad/s^2)');
grid on

figure;
plot(t,gait_f2(33,:));
title('t-ddq5');xlabel('t(s)');ylabel('ddq5(rad/s^2)');
grid on

figure;
plot(t,gait_f2(34,:));
title('t-ddq6');xlabel('t(s)');ylabel('ddq6(rad/s^2)');
grid on

figure;
plot(t,gait_f2(35,:));
title('t-ddq7');xlabel('t(s)');ylabel('ddq7(rad/s^2)');
grid on

figure;
plot(t,gait_f2(36,:));
title('t-ddq8');xlabel('t(s)');ylabel('ddq8(rad/s^2)');
grid on



%% VERIFY THE DIFFERENCE BETWEEN DIFF AND J
% for i = 1:8
% figure;
% plot(t,gait_f2_test(i,1:end-2));
% grid on
% end
% for i = 1:8
% figure;
% plot(t,gait_f2_test_2(i,1:end-2));
% grid on
% end


%% verify 
% figure;
% plot(t(1:end-1),diff(gait_f2(20,:))/dt*pi/180);
% title('t-dq8');xlabel('t(s)');ylabel('dq8(rad/s)');
% grid on
% figure;
% plot(t(1:end-1),diff(gait_f2(28,:))/dt);
% title('t-ddq8');xlabel('t(s)');ylabel('ddq8(rad/s^2)');
% grid on