% PLOTTING
clc; clear all; close all;
global Kx Ky Kz Kphi Ktheta Kpsi Jphi Jtheta Jpsi m g Ar mar_stable num
global pv1 pv2 pv3 Local
load multi_UAV.mat;
load stocastic_item.mat;
load UAV1.mat;
load UAV2.mat;
load UAV3.mat;
load UAV4.mat;
plotN = 15000;
h = T/L ;

figure(1)
subplot(2,3,1)
plot(0:h:h*(plotN-1),r1(1,1:plotN),'black')
axis([0 30 -2 2])
magnify()
hold on
plot(0:h:h*(plotN-1),Xt1(1,1:plotN),'b')
hold on
plot(0:h:h*(plotN-1),Xt1(13,1:plotN),'r')
hold on
plot(0:h:h*(plotN-1),Xt2(13,1:plotN),'g')
hold on
plot(0:h:h*(plotN-1),Xt3(13,1:plotN),'y')
hold on
plot(0:h:h*(plotN-1),Xt4(13,1:plotN),'m')
legend('desired x(t)','leader x(t)','follower1 x(t)',...
    'follower2 x(t)','follower3 x(t)','follower4 x(t)');
title('x trajecctory');
subplot(2,3,2)
plot(0:h:h*(plotN-1),r1(3,1:plotN),'black')
axis([0 30 -2 2])
hold on
plot(0:h:h*(plotN-1),Xt1(3,1:plotN),'b')
hold on
plot(0:h:h*(plotN-1),Xt1(15,1:plotN),'r')
hold on
plot(0:h:h*(plotN-1),Xt2(15,1:plotN),'g')
hold on
plot(0:h:h*(plotN-1),Xt3(15,1:plotN),'y')
hold on
plot(0:h:h*(plotN-1),Xt4(15,1:plotN),'m')
legend('desired y(t)','leader y(t)','follower1 y(t)',...
    'follower2 y(t)','follower3 y(t)','follower4 y(t)');
title('y trajecctory');
subplot(2,3,3)
plot(0:h:h*(plotN-1),r1(5,1:plotN),'black')
hold on
plot(0:h:h*(plotN-1),Xt1(5,1:plotN),'b')
hold on
plot(0:h:h*(plotN-1),Xt1(17,1:plotN),'r')
hold on
plot(0:h:h*(plotN-1),Xt2(17,1:plotN),'g')
hold on
plot(0:h:h*(plotN-1),Xt3(17,1:plotN),'y')
hold on
plot(0:h:h*(plotN-1),Xt4(17,1:plotN),'m')
legend('desired z(t)','leader z(t)','follower1 z(t)',...
    'follower2 z(t)','follower3 z(t)','follower4 z(t)');
title('z trajecctory');
subplot(2,3,4)
plot(0:h:h*(plotN-1),r1(2,1:plotN),'black')
axis([0 30 -2 2])
hold on
plot(0:h:h*(plotN-1),Xt1(2,1:plotN),'b')
hold on
plot(0:h:h*(plotN-1),Xt1(14,1:plotN),'r')
hold on
plot(0:h:h*(plotN-1),Xt2(14,1:plotN),'g')
hold on
plot(0:h:h*(plotN-1),Xt3(14,1:plotN),'y')
hold on
plot(0:h:h*(plotN-1),Xt4(14,1:plotN),'m')
legend('desired Vx(t)','leader Vx(t)','follower1 Vx(t)',...
    'follower2 Vx(t)','follower3 Vx(t)','follower4 Vx(t)');
title('Vx trajecctory');
subplot(2,3,5)
plot(0:h:h*(plotN-1),r1(4,1:plotN),'black')
axis([0 30 -2 2])
hold on
plot(0:h:h*(plotN-1),Xt1(4,1:plotN),'b')
hold on
plot(0:h:h*(plotN-1),Xt1(16,1:plotN),'r')
hold on
plot(0:h:h*(plotN-1),Xt2(16,1:plotN),'g')
hold on
plot(0:h:h*(plotN-1),Xt3(16,1:plotN),'y')
hold on
plot(0:h:h*(plotN-1),Xt4(16,1:plotN),'m')
legend('desired Vy(t)','leader Vy(t)','follower1 Vy(t)',...
    'follower2 Vy(t)','follower3 Vy(t)','follower4 Vy(t)');
title('Vy trajecctory');
subplot(2,3,6)
plot(0:h:h*(plotN-1),r1(6,1:plotN),'black')
hold on
plot(0:h:h*(plotN-1),Xt1(6,1:plotN),'b')
hold on
plot(0:h:h*(plotN-1),Xt1(18,1:plotN),'r')
hold on
plot(0:h:h*(plotN-1),Xt2(18,1:plotN),'g')
hold on
plot(0:h:h*(plotN-1),Xt3(18,1:plotN),'y')
hold on
plot(0:h:h*(plotN-1),Xt4(18,1:plotN),'m')
legend('desired Vz(t)','leader Vz(t)','follower1 Vz(t)',...
    'follower2 Vz(t)','follower3 Vz(t)','follower4 Vz(t)');
title('Vz trajecctory');

figure(2)
subplot(2,3,1)
plot(0:h:h*(plotN-1),r1(7,1:plotN),'black')
axis([0 30 -45 45])
hold on
plot(0:h:h*(plotN-1),Xt1(7,1:plotN),'b')
hold on
plot(0:h:h*(plotN-1),Xt1(19,1:plotN),'r')
hold on
plot(0:h:h*(plotN-1),Xt2(19,1:plotN),'g')
hold on
plot(0:h:h*(plotN-1),Xt3(19,1:plotN),'y')
hold on
plot(0:h:h*(plotN-1),Xt4(19,1:plotN),'m')
legend('desired phi(t)','leader phi(t)','follower1 phi(t)',...
    'follower2 phi(t)','follower3 phi(t)','follower4 phi(t)');
title('phi trajecctory');
subplot(2,3,2)
plot(0:h:h*(plotN-1),r1(9,1:plotN),'black')
axis([0 30 -45 45])
hold on
plot(0:h:h*(plotN-1),Xt1(9,1:plotN),'b')
hold on
plot(0:h:h*(plotN-1),Xt1(21,1:plotN),'r')
hold on
plot(0:h:h*(plotN-1),Xt2(21,1:plotN),'g')
hold on
plot(0:h:h*(plotN-1),Xt3(21,1:plotN),'y')
hold on
plot(0:h:h*(plotN-1),Xt4(21,1:plotN),'m')
legend('desired theta(t)','leader theta(t)','follower1 theta(t)',...
    'follower2 theta(t)','follower3 theta(t)','follower4 theta(t)');
title('theta trajecctory');
subplot(2,3,3)
plot(0:h:h*(plotN-1),r1(11,1:plotN),'black')
axis([0 30 -0.1 0.1])
hold on
plot(0:h:h*(plotN-1),Xt1(11,1:plotN),'b')
hold on
plot(0:h:h*(plotN-1),Xt1(23,1:plotN),'r')
hold on
plot(0:h:h*(plotN-1),Xt2(23,1:plotN),'g')
hold on
plot(0:h:h*(plotN-1),Xt3(23,1:plotN),'y')
hold on
plot(0:h:h*(plotN-1),Xt4(23,1:plotN),'m')
legend('desired psi(t)','leader psi(t)','follower1 psi(t)',...
    'follower2 psi(t)','follower3 psi(t)','follower4 psi(t)');
title('psi trajecctory');
subplot(2,3,4)
plot(0:h:h*(plotN-1),r1(8,1:plotN),'black')
%axis([0 30 -2 2])
hold on
plot(0:h:h*(plotN-1),Xt1(8,1:plotN),'b')
hold on
plot(0:h:h*(plotN-1),Xt1(20,1:plotN),'r')
hold on
plot(0:h:h*(plotN-1),Xt2(20,1:plotN),'g')
hold on
plot(0:h:h*(plotN-1),Xt3(20,1:plotN),'y')
hold on
plot(0:h:h*(plotN-1),Xt4(20,1:plotN),'m')
legend('desired Vphi(t)','leader Vphi(t)','follower1 Vphi(t)',...
    'follower2 Vphi(t)','follower3 Vphi(t)','follower4 Vphi(t)');
title('Vphi trajecctory');
subplot(2,3,5)
plot(0:h:h*(plotN-1),r1(10,1:plotN),'black')
%axis([0 30 -2 2])
hold on
plot(0:h:h*(plotN-1),Xt1(10,1:plotN),'b')
hold on
plot(0:h:h*(plotN-1),Xt1(22,1:plotN),'r')
hold on
plot(0:h:h*(plotN-1),Xt2(22,1:plotN),'g')
hold on
plot(0:h:h*(plotN-1),Xt3(22,1:plotN),'y')
hold on
plot(0:h:h*(plotN-1),Xt4(22,1:plotN),'m')
legend('desired Vtheta(t)','leader Vtheta(t)','follower1 Vtheta(t)',...
    'follower2 Vtheta(t)','follower3 Vtheta(t)','follower4 Vtheta(t)');
title('Vtheta trajecctory');
subplot(2,3,6)
plot(0:h:h*(plotN-1),r1(12,1:plotN),'black')
axis([0 30 -1 1])
hold on
plot(0:h:h*(plotN-1),Xt1(12,1:plotN),'b')
hold on
plot(0:h:h*(plotN-1),Xt1(24,1:plotN),'r')
hold on
plot(0:h:h*(plotN-1),Xt2(24,1:plotN),'g')
hold on
plot(0:h:h*(plotN-1),Xt3(24,1:plotN),'y')
hold on
plot(0:h:h*(plotN-1),Xt4(24,1:plotN),'m')
legend('desired Vpsi(t)','leader Vpsi(t)','follower1 Vpsi(t)',...
    'follower2 Vpsi(t)','follower3 Vpsi(t)','follower4 Vpsi(t)');
title('Vpsi trajecctory');


%[Poisson1 Poisson2 Poisson3 Poisson4 Poisson5]=mod_Poisson(L+1,Counting1,Counting2,Counting3,Counting4,Counting5);
figure(3)
plot(0:T/L:T,Counting1,'b')
hold on
plot(0:T/L:T,Counting2,'r')
hold on
plot(0:T/L:T,Counting3,'g')
hold on
plot(0:T/L:T,Counting4,'y')
hold on
plot(0:T/L:T,Counting5,'m')
xlabel('Time(sec)')
title('Poisson counting process of the UAVs')

[Weiner1 Weiner2 Weiner3 Weiner4 Weiner5]=mod_Weiner(L,dW1,dW2,dW3,dW4,dW5);
plotN = 1000;
figure(4)
plot(0:h:h*(plotN-1),Weiner1(1:plotN),'b')
hold on
plot(0:h:h*(plotN-1),Weiner2(1:plotN),'r')
hold on
plot(0:h:h*(plotN-1),Weiner3(1:plotN),'g')
hold on
plot(0:h:h*(plotN-1),Weiner4(1:plotN),'y')
hold on
plot(0:h:h*(plotN-1),Weiner5(1:plotN),'m')
xlabel('Time(sec)')
title('Weiner process of the UAVs')