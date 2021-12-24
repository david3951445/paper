% Runge-Kutta-Fehlberg method on nonlinear stochastic ONE UAV tracking model
clc; clear all; close all;
global Kx Ky Kz Kphi Ktheta Kpsi Jphi Jtheta Jpsi m g Ar mar_stable num
global pv1 pv2 pv3 Local h
load multi_UAV.mat;
load stocastic_item.mat;
L=15000;
T=L*h;
Xt_L=[0.54 0.55 0.57 0.57 2 0.5	0.51 0.59 0.52 0.52 0.55 0.52]';
Xt1 = [Xt_L;Xt_F1];

Xt1 = [Xt1 zeros(length(Xt1),L-1)];
r1 = zeros(12,L);
r2 = zeros(12,L);
%% Leader + Follower 1 DE
for j = 1:L
%     Winc1 = sum(dW1(R*(j-1)+1:R*j));
%     Winc2 = sum(dW2(R*(j-1)+1:R*j));
    if j==1
    [k1 kr1 kr2 d1] = multi_UAV_DE_1(j*h,Xt1(:,j),index,Xt1(:,j));
    [k2 kr0 kr0 d2] = multi_UAV_DE_1(j*h+h/2,Xt1(:,j)+h*k1/2,index,Xt1(:,j));
    [k3 kr0 kr0 d3] = multi_UAV_DE_1(j*h+h/2,Xt1(:,j)+h*k2/2,index,Xt1(:,j));
    [k4 kr0 kr0 d4] = multi_UAV_DE_1(j*h+h,Xt1(:,j)+h*k3,index,Xt1(:,j));
    else
     [k1 kr1 kr2 d1] = multi_UAV_DE_1(j*h,Xt1(:,j),index,r1(:,j-1));
    [k2 kr2 kr0 d2] = multi_UAV_DE_1(j*h+h/2,Xt1(:,j)+h*k1/2,index,r1(:,j-1));
    [k3 kr3 kr0 d3] = multi_UAV_DE_1(j*h+h/2,Xt1(:,j)+h*k2/2,index,r1(:,j-1));
    [k4 kr4 kr0 d4] = multi_UAV_DE_1(j*h+h,Xt1(:,j)+h*k3,index,r1(:,j-1));
    end
    Xt1(:,j+1)=Xt1(:,j)+h*(k1+2*k2+2*k3+k4)/6;
    r1(:,j+1)=kr1;
    %% weiner
%     Xt1(:,j+1)=Xt1(:,j+1)+[0;Xt1(2,j+1);0;Xt1(4,j+1);0;Xt1(6,j+1);0;Xt1(8,j+1);0;Xt1(10,j+1);0;Xt1(12,j+1);zeros(12,1)]*(Winc1/10)...
%                          +[zeros(12,1);0;Xt1(14,j+1);0;Xt1(16,j+1);0;Xt1(18,j+1);0;Xt1(20,j+1);0;Xt1(22,j+1);0;Xt1(24,j+1)]*(Winc2/10);    
%       %% poisson
%     if j*h<=Times1(c1)+2*h && j*h>Times1(c1)
%         Xt1(:,j+1)=Xt1(:,j+1)+[0;Xt1(2,j+1);0;Xt1(4,j+1);0;Xt1(6,j+1);0;Xt1(8,j+1);0;Xt1(10,j+1);0;Xt1(12,j+1);zeros(12,1)]*1/2;
if j>1
r1(8,j)=(r1(7,j)-r1(7,j-1))/h;
r1(10,j)=(r1(9,j)-r1(9,j-1))/h;
    
end
     
    

    d(:,j+1) = d1;
   
    j1=j
end
% for j = 0:L
%    if j*h<=Times1(dp1)+1.5*h && j*h>Times1(dp1)
%        level1 = level1 + 1;
%        Counting1(ep1) = level1;
%        ep1 = ep1 + 1;
%        dp1 = dp1 + 1;
%    else
%        Counting1(ep1) = level1;
%        ep1 = ep1 + 1;
%    end
%    if j*h<=Times2(dp2)+1.5*h && j*h>Times2(dp2)
%        level2 = level2 + 1;
%        Counting2(ep2) = level2;
%        ep2 = ep2 + 1;
%        dp2 = dp2 + 1;
%    else
%        Counting2(ep2) = level2;
%        ep2 = ep2 + 1;
%    end
% end
%% Plot
close all

plotN = L;
figure(1)
subplot(1,3,1)
plot(0:h:h*(plotN-1),r1(1,1:plotN),'black')
axis([0 T -20 20])

hold on
plot(0:h:h*(plotN-1),Xt1(1,1:plotN),'b')
leg4=legend('Desired $x$(t)','$x$(t) in [27]');
set(leg4,'Interpreter','latex');
leg5=title('$x$ Trajecctory');
set(leg5,'Interpreter','latex');
subplot(1,3,2)
plot(0:h:h*(plotN-1),r1(3,1:plotN),'black')
axis([0 T -20 20])
hold on
plot(0:h:h*(plotN-1),Xt1(3,1:plotN),'b')
hold on
leg4=legend('Desired $y$(t)','$y$(t) in [27]');
set(leg4,'Interpreter','latex');
leg5=title('$y$ Trajecctory');
set(leg5,'Interpreter','latex');
subplot(1,3,3)
plot(0:h:h*(plotN-1),r1(5,1:plotN),'black')
hold on
plot(0:h:h*(plotN-1),Xt1(5,1:plotN),'b')
hold on
axis([0 T -20 60])
leg4=legend('Desired $z$(t)','$z$(t) in [27]');
set(leg4,'Interpreter','latex');
leg5=title('$z$ Trajecctory');
set(leg5,'Interpreter','latex');


figure (2)
subplot(1,3,1)
plot(0:h:h*(plotN-1),r1(2,1:plotN),'black')
hold on
plot(0:h:h*(plotN-1),Xt1(2,1:plotN),'b')
axis([0 T -20 20])
leg1=legend('desired $V_{x}$(t)','$V_{x}$(t) in [27]');
set(leg1,'Interpreter','latex');
leg2=title('$V_{x}$ Trajecctory');
set(leg2,'Interpreter','latex');
subplot(1,3,2)
plot(0:h:h*(plotN-1),r1(4,1:plotN),'black')
hold on
plot(0:h:h*(plotN-1),Xt1(4,1:plotN),'b')
leg2=legend('desired $V_{y}$(t)','$V_{y}$(t) in [27]');
set(leg2,'Interpreter','latex');
 set(leg2,'FontSize',9);
axis([0 T -20 20])
leg2=title('$V_{y}$ Trajecctory');
set(leg2,'Interpreter','latex');
subplot(1,3,3)
plot(0:h:h*(plotN-1),r1(6,1:plotN),'black')
hold on
plot(0:h:h*(plotN-1),Xt1(6,1:plotN),'b')
leg3=legend('desired $V_{z}$(t)','$V_{z}$(t) in [27]');
set(leg3,'Interpreter','latex');
 set(leg3,'FontSize',9);
axis([0 T -20 20])
leg2=title('$V_{z}$ Trajecctory');
set(leg2,'Interpreter','latex');

figure(3)
subplot(1,3,1)
plot(0:h:h*(plotN-1),r1(7,1:plotN),'black')
axis([0 T -20 20])
hold on
plot(0:h:h*(plotN-1),Xt1(7,1:plotN),'b')
leg4=legend('Desired $\theta$(t)','$\theta$(t) in [27]');
set(leg4,'Interpreter','latex');
leg5=title('$\theta$ Trajecctory');
set(leg5,'Interpreter','latex');
subplot(1,3,2)
plot(0:h:h*(plotN-1),r1(9,1:plotN),'black')
axis([0 T -20 20])
hold on
plot(0:h:h*(plotN-1),Xt1(9,1:plotN),'b')
leg4=legend('Desired $\psi$(t)','$\psi$(t) in [27]');
set(leg4,'Interpreter','latex');
leg5=title('$\psi$ Trajecctory');
set(leg5,'Interpreter','latex');
subplot(1,3,3)
plot(0:h:h*(plotN-1),r1(11,1:plotN),'black')
axis([0 T -0.1 0.2])
hold on
plot(0:h:h*(plotN-1),Xt1(11,1:plotN),'b')
leg4=legend('Desired $\phi$(t)','$\phi$(t) in [27]');
set(leg4,'Interpreter','latex');
leg5=title('$\phi$ Trajecctory');
set(leg5,'Interpreter','latex');


figure (4)
subplot(1,3,1)
plot(0:h:h*(plotN-1),r1(8,1:plotN),'black')
axis([0 T -20 20])
hold on
plot(0:h:h*(plotN-1),Xt1(8,1:plotN),'b')
leg4=legend('Desired $V_{\theta}$(t)','$V_{\theta}$(t) in [27]');
set(leg4,'Interpreter','latex');
leg5=title('$V_{\theta}$ Trajecctory');
set(leg5,'Interpreter','latex');
subplot(1,3,2)
plot(0:h:h*(plotN-1),r1(10,1:plotN),'black')
axis([0 T -20 20])
hold on
plot(0:h:h*(plotN-1),Xt1(10,1:plotN),'b')
leg4=legend('Desired $V_{\psi}$(t)','$V_{\psi}$(t) in [27]');
set(leg4,'Interpreter','latex');
leg5=title('$V_{\psi}$ Trajecctory');
set(leg5,'Interpreter','latex');
subplot(1,3,3)
plot(0:h:h*(plotN-1),r1(12,1:plotN),'black')
axis([0 T -20 20])
hold on
plot(0:h:h*(plotN-1),Xt1(12,1:plotN),'b')

leg4=legend('Desired $V_{\phi}$(t)','$V_{\phi}$(t) in [27]');
set(leg4,'Interpreter','latex');
leg5=title('$V_{\phi}$ Trajecctory');
set(leg5,'Interpreter','latex');
figure (5)
plot3(r1(1,1:plotN),r1(3,1:plotN),r1(5,1:plotN),'black')
grid on;
hold on
plot3(Xt1(1,1:plotN),Xt1(3,1:plotN),Xt1(5,1:plotN),'b')
hold on
legend('desired trajectory','UAV 3-D trajectory in [27]');




save UAV1.mat;
save okdata.mat
