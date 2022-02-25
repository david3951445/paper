clc;clear;
close all
h = 1e-4;
t = [0;1;2;4;5;6;7];
x1 = [1;2;3;4;4.5;5;6];
y1 = [1;2;2;2;3;4;5];
tt = 0:h:t(end);
l = length(tt);
d = 0.5;
%%% linear velocity 
xx1 = spline(t,x1,tt);
yy1 = spline(t,y1,tt);
vx1 = diff(xx1)/h;
vy1 = diff(yy1)/h;
ax1 = diff(vx1)/h;
ay1 = diff(vy1)/h;
jx1 = diff(ax1)/h;
jy1 = diff(ay1)/h;
%%% angular velocity 
th1 = atan2d(vy1,vx1);
w1 = diff(th1)/h;
afa1 = diff(w1)/h;

figure; plot(tt(1:end-1),vx1); title('tt-vx1');
figure; plot(tt(1:end-1),vy1); title('tt-vy1');
figure; plot(xx1,yy1); title('xx1-yy1');
v = (vx1.^2+vy1.^2).^(1/2);
figure; plot(tt(1:end-1),v); title('tt-v');
hf = 0.1; %% half of max height since the abs(z)
c = 0.5; %% the constant multiply to v --> deteremine the gait length
T = 0.5; %% time period of a single step
num_step = 7/(2*T); %% the number iof step during walking 
%%% one step gait (fixed velocity) %%%
% gait = leg_gait_test(2);
%%% one step gait (changed velocity) %%%  
% x = linspace(0,pi,8) % initial value, final value, n=no. of po
th = linspace(-pi/2,3*pi/2+(2*pi*num_step),l);
for i = 1:length(v)
    gait(:,i) = leg_gait(v(i),th(i),T);
end
figure;
plot(gait(1,:),gait(4,:));
title('x-z');
axis equal
for i = 1:6
    figure;
    plot(tt(1:end-1),gait(i,:));
end
%%% diff() %%%
% figure;
% plot(tt(1:end-1),gait(4,:));
% title('z-t (diff)');
% vz = diff(gait(4,:))/h;
% figure;
% plot(tt(1:end-2),vz);
% title('vz-t (diff)');
% figure; 
% plot(tt(1:end-3),diff(diff(gait(4,:))/h)/h);
% title('az-t (diff)');
% figure; 
% plot(tt(1:end-2),diff(gait(1,:))/h);
% title('vx-t (diff)');
% figure; 
% plot(tt(1:end-3),diff(diff(gait(1,:))/h)/h);
% title('ax-t (diff)');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Conclusion:
% the velocity and acceleration are different between leg_gait() and diff()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
