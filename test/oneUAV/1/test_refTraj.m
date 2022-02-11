% test of r(t)
clc; clear; close all;
L=15000;
h=0.002;
T=L*h;
for j = 1:L    
    r1(:,j+1) = fun(j*h, Xt1(:,j));
end

plot(0:h:h*(L-1),r1(7, 1:L),'black')
axis([0 T -20 20])
hold on

function r1 = fun(t, x)
radius=10;
freq=0.5;
r1(1) = radius*sin(freq*t);
r1(2) = freq*radius*cos(freq*t);
r1(3) = radius*cos(freq*t);
r1(4) = -freq*radius*sin(freq*t);
r1(5) = 1*t;
r1(6) = 1;
%Orginal Method
d1(1) = (x(1)-r1(1));  %% x-xd
d1(2) = x(2)-r1(2);    %% x_dot-x_dotd
d1(3) = (x(3)-r1(3));  %% y-yd
d1(4) = x(4)-r1(4);    %% y_dot-y_dotd
ux = 5*d1(1)+1.5*d1(2);
uy = 5*d1(3)+1.5*d1(4);

r1(7) = 20*atan(ux*sin(r1(11))-1*uy*cos(r1(11)));                 %% roll
%   r1(8) = (r1(7)-f1(7))/h;
r1(9) = -30*atan((ux*cos(r1(11))+uy*sin(r1(11)))/cos(r1(11)));  %% pitch
%   r1(10) = (r1(9)-f1(9))/h;
r1(11) = 0;  %% pitch
end