clc;clear;
close all
h = 0.0001;
t = [0;1;2;4;5];
x1 = [1;3;4;5;7];
y1 = [2;4;3;6;7];
tt = 0:h:5;
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

len = 4001;
% figure; plot(tt(1:end-1),vx1); title('tt-vx1');
% figure; plot(tt(1:end-1),vy1); title('tt-vy1');
% figure; plot(xx1(1:len),yy1(1:len)); title('xx1-yy1');
v = (vx1.^2+vy1.^2).^(1/2);
% figure; plot(tt(1:end-1),v); title('tt-v');
S = 1; %% stride
T = 0.5; %% time period of a single step
hf = 0.1; %% half of max height since the abs(z)
c = 0.5; %% the constant multiply to v
%%%% (I) it is assumed that the robot walks on the straight line.
%%%%     Therefore, the direction of leg(i) points to x-axis of body local
%%%%     frame.
for i = 1:length(tt)-1
    leg(i) = c*v(i)*sin(pi/T*(tt(i)-T/2));
    vleg2(i) = c*v(i)*cos(pi/T*(tt(i)-T/2))*pi/T;
end
for i = 1:length(tt)-1
    legl(i) = c*v(i)*sin(pi/T*(tt(i)+T/2));
end

vleg = diff(leg)/h;
figure
plot(tt(1:end-2),vleg)
figure
plot(tt(1:end-1),vleg2)
vv = v(1:end-1);
n=5;
p = polyfit(vleg,vv,n) ;
vv2 = polyval(p,vleg);
figure
plot(tt(1:end-2),vv,tt(1:end-2),vv2)
%%%% whether or not to use zq? Since the trajectory and velocity of z is
%%%% unvarianat 
for i = 1:length(tt)-1
    if rem(floor(tt(i)/T),2) == 0
        z(i) = hf/2*(1+sin(2*pi/T*(tt(i)-T/4)));
    else
        z(i) = 0;
    end
end
for i = 1:length(tt)-1
    if rem(floor(tt(i)/T),2) == 1
        zl(i) = hf/2*(1+sin(2*pi/T*(tt(i)-T/4)));
    else
        zl(i) = 0;
    end
end
%%% if tt == ttq, then the result of spline is same as original result.
zq = spline(tt(1:end-1),z,tt(1:end-1));
figure;plot(tt(1:end-1),leg); title('t - leg position(x)');
% figure;plot(leg,zq); title('leg position(x) - leg position(z)')
figure;plot(tt(1:end-1),zq); title('t - leg position(z)')
% 
% 
% vz = diff(z)/h;
% figure;plot(tt(1:end-2),vz); title('t - leg velocity(vz)')
% az = diff(vz)/h;
% figure;plot(tt(1:end-3),az); title('t - leg acc(az)')
% figure; comet(leg,zq);
% figure; comet(legl,zl); %% left leg
