%%%%%%% conclusion %%%%%%%%%%%%%%%
%%% holomonic constraint can be used in the equation %%%
close all
clc;clear;
load ReferenceTrajectoryTest.mat
%%%%% deal with the matrix dimension %%%%%
ttr = tt(1:len);
disp(ttr(end));
%%%%%%% trailer %%%%%%%%%%%%%%%
% %%% linear velocity %%%
% figure;
% plot(t(1:4),x(1:4),'o',ttr,xx1(1:len));
% figure;
% plot(t(1:4),y(1:4),'o',ttr,yy1(1:len));
% figure;
% plot(ttr,vx1(1:len));
% figure;
% plot(ttr,vy1(1:len));
% figure;
% plot(ttr,ax1(1:len));
% figure;
% plot(ttr,ay1(1:len));
% figure;
% plot(ttr,jx1(1:len));
% figure;
% plot(ttr,jy1(1:len));
%%% angular velocity %%%
% figure;
% plot(xx1(1:len),yy1(1:len));
% figure;
% plot(ttr,th1(1:len));
% figure;
% plot(ttr,w1(1:len));
% figure;
% plot(ttr,afa1(1:len));
%%%%%%% tractor %%%%%%%%%%%%%%%
%%% linear velocity %%%
% figure;
% plot(ttr,xx0(1:len));
% figure;
% plot(ttr,yy0(1:len));
% figure;
% plot(ttr,vx0(1:len));
% figure;
% plot(ttr,vy0(1:len));
% figure;
% plot(ttr,ax0(1:len));
% figure;
% plot(ttr,ay0(1:len));
% figure;
% plot(ttr,jx0(1:len));
% figure;
% plot(ttr,jy0(1:len));
%%% angular velocity %%%
figure;
plot(xx1(1:len),yy1(1:len),xx0(1:len),yy0(1:len));
xlabel('x-axis coordinate (m)');
ylabel('y-axis coordinate (m)');
title('reference path of the tractor-trailer');
legend('trailer','tractor');
grid on
% figure;
% plot(ttr,th0(1:len));
% figure;
% plot(ttr,w0(1:len));
% figure;
% plot(ttr,afa0(1:len));
%%%%%%% comparison %%%%%%%%%%%%%%%
figure;
plot(ttr,vx1(1:len),ttr,vx2(1:len));
figure;
plot(ttr,vy1(1:len),ttr,vy2(1:len));
figure;
plot(ttr,w1(1:len),ttr,w2(1:len));