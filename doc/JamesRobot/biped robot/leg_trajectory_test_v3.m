clc;clear;

S = 1; %% stride
T = 2; %% time period of a single step
hf = 0.25; %% half of max height since the abs(z)
T1 = 1; %% speed up
%% h & t
h = 0.005;
h1 = 0.1;
t = 0:h:6*T;

%% x
x = S*sin(pi/T*(t-T/2));
%% z
for i = 1:length(t)
    if rem(floor(t(i)/T),2) == 0
        z(i) = hf/2*(1+sin(2*pi/T*(t(i)-T/4)));
    else
        z(i) = 0;
    end
end
%% vx
vx = S*cos(pi/T*(t-T/2))*pi/T;
for i = 1:length(t)
    SS(i) = vx(i)/(cos(pi/T*(t(i)-T/2))*pi/T);
end
%% vz
vz = diff(z)/h;
%% spline
hq = 0.001;
tq = 0:hq:6*T;
zq = spline(t,z,tq);
% xq = spline(t,x,tq);
xq = S*sin(pi/T*(tq-T/2));
vxq = diff(xq)/hq;
vzq = diff(zq)/hq;
axq = diff(vxq)/hq;
azq = diff(vzq)/hq;
len_v = length(tq)-1;
len_a = length(tq)-2;
tq_v = tq(1:len_v);
tq_a = tq(1:len_a);
for i = 1:length(tq)-1
    Sq(i) = vxq(i)/(cos(pi/T*(tq(i)-T/2))*pi/T);
end
save leg_trajectory_test_v3.mat



