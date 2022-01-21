clc;clear;

S = 1; %% stride
T = 2; %% time period of a single step
hf = 0.25; %% half of max height since the abs(z)
T1 = 1; %% speed up
%% h & t
h = 0.05;
h1 = 0.1;
t = 0:h:6*T;
t1 = 0:h1:T1;

%% x
% x = S*sin(pi/T*(t-T/2));

for i = 1:length(t)
    if rem(floor(t(i)/T),2) == 0
        x(i) = -S/2 + S/T*(t(i)-floor(t(i)/T)*T);
    else
        x(i) = S/2 - S/T*(t(i)-floor(t(i)/T)*T);
    end
end

x1 = S/T1*t1 - S/2;
%% z
% for i = 1:length(t)
%     if rem(floor(t(i)/T),2) == 0
%         z(i) = hf/2*(1+sin(2*pi/T*(t(i)-T/4)));
%     else
%         z(i) = 0;
%     end
% end

% z = hf/2*(1+sin(2*pi/T*(t-T/4)));
z = hf*sin(pi/T*t) + abs(hf*sin(pi/T*t));
z1 = hf*sin(pi/T1*t1);
%% vx
% vx = S*cos(pi/T*(t-T/2))*pi/T;
vx = S/T*ones(1,length(t));
vx1 = S/T1*ones(1,length(t1));
% z1 = hf*sin(pi/T1*t1);
%% spline
hq = 0.001;
tq = 0:hq:3*T;
zq = spline(t,z,tq);
xq = spline(t,x,tq);
vxq = diff(xq)/hq;
vzq = diff(zq)/hq;
axq = diff(vxq)/hq;
azq = diff(vzq)/hq;
len_v = length(tq)-1;
len_a = length(tq)-2;
tq_v = tq(1:len_v);
tq_a = tq(1:len_a);
%% the effect of change of vx to gait
vxc = [0.5 0.3 0.5 0.3 0.7 0.4];
tc = 0:5;
Tc = S./vxc;
tqc = 0:hq:5;
vxcq = spline(tc,vxc,tqc);
Tcq = S./vxcq;

for i = 1:length(tqc)
%     if rem(floor(tqc(i)/Tcq(i)),2) == 0
%         xc(i) = -S/2 + S/Tcq(i)*(tqc(i)-floor(tqc(i)/Tcq(i))*Tcq(i));
%     else
%         xc(i) = S/2 - S/Tcq(i)*(tqc(i)-floor(tqc(i)/Tcq(i))*Tcq(i));
%     end
    xc(i) = S/Tcq(i)*tqc(i);
end

zc = hf*sin(pi./T.*tqc) + abs(hf*sin(pi./Tcq.*tqc));


save leg_trajectory_test_v2.mat



