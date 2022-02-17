% PLOTTING 3D figure
clc; clear all; close all;
global Kx Ky Kz Kphi Ktheta Kpsi Jphi Jtheta Jpsi m g Ar mar_stable num
global pv1 pv2 pv3 Local
load multi_UAV.mat;
load stocastic_item.mat;
load UAV2.mat;
load UAV3.mat;
load UAV4.mat;
load UAV1.mat;

plotN = 15000;
figure(7)
plot3(r1(1,1:plotN),r1(3,1:plotN),r1(5,1:plotN),'black')
axis([-2 2 -2 2 0 30])
grid on;
hold on
plot3(Xt1(1,1:plotN),Xt1(3,1:plotN),Xt1(5,1:plotN),'b')
hold on
plot3(Xt1(13,1:plotN),Xt1(15,1:plotN),Xt1(17,1:plotN),'r')
hold on
plot3(Xt2(13,1:plotN),Xt2(15,1:plotN),Xt2(17,1:plotN),'g')
hold on
plot3(Xt3(13,1:plotN),Xt3(15,1:plotN),Xt3(17,1:plotN),'y')
hold on
plot3(Xt4(13,1:plotN),Xt4(15,1:plotN),Xt1(17,1:plotN),'m')
legend('desired trajectory','leader trajectory','follower1 trajectory',...
    'follower2 trajectory','follower3 trajectory','follower4 trajectory');
