clc;clear;
close all
h = 1e4;
%%% when time period of one gait is 2*T %%%
%%% 0 ~ 2T is matching to pi ~ -pi %%% 
T = 0.5; %% time period of a single step
Tq = linspace(0,2*T,h);
dt = Tq(2)-Tq(1);
%%% trajectory design %%%
t = [0;1;2;4;5;6;7];
x1 = [1;2;3;4;4.5;5;6];
y1 = [1;2;2;2;3;4;5];

num_step = 7/(2*T);
tt = linspace(0,7,num_step*h);
l = length(tt);
d = 0.5;

% linear velocity
xx1 = spline(t,x1,tt);
yy1 = spline(t,y1,tt);
vx1 = diff(xx1)/dt;
vy1 = diff(yy1)/dt;
ax1 = diff(vx1)/dt;
ay1 = diff(vy1)/dt;
jx1 = diff(ax1)/dt;
jy1 = diff(ay1)/dt;
% angular velocity 
th1 = atan2d(vy1,vx1);
w1 = diff(th1)/dt;
afa1 = diff(w1)/dt;

figure; plot(tt(1:end-1),vx1); title('tt-vx1');
figure; plot(tt(1:end-1),vy1); title('tt-vy1');
figure; plot(xx1,yy1); title('xx1-yy1');
v = (vx1.^2+vy1.^2).^(1/2);
% figure; plot(tt(1:end-1),v); title('tt-v');

hf = 1; %% half of max height since the abs(z)
c = 0.5; %% the constant multiply to v --> deteremine the gait length
 %% the number of step during walking 
%%% one step gait (fixed velocity) %%%
% gait = leg_gait_test(2);
%%% one step gait (changed velocity) %%%  
% x = linspace(0,pi,8) % initial value, final value, n=no. of po
for i = 1:length(v)
    if mod(i,h) ~= 0
        index = i - floor(i/h)*h;
    else 
        index = (floor(i/h)-(floor(i/h)-1))*h;
    end
    gait(:,i) = leg_gait_s(hf,c,h,v(i),index,dt);
end

save leg_trajectory_test_v6.mat


