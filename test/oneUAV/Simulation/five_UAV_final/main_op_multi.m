% Runge-Kutta-Fehlberg method on nonlinear stochastic ONE UAV tracking model
clc; clear all; close all;
global Kx Ky Kz Kphi Ktheta Kpsi Jphi Jtheta Jpsi m g Ar mar_stable num
global pv1 pv2 pv3 Local
%% system parameter %%
Kx = 0.01;
Ky = 0.01;
Kz = 0.01;
Kphi = 0.012;
Ktheta = 0.012;
Kpsi = 0.012;
Jphi = 7.5e-3;
Jtheta = 7.5e-3;
Jpsi = 13e-3;
m = 0.65;
g = 9.81;
Ar = -eye(12);
num = 2;
%% system parameter END %%

%% 家览丁把计 ㎝ 繦诀把计 %%
dt = 1e-3;T = 5; N = T/dt; 
R = 2; Dt = R*dt; L = N/R; % L steps size Dt = R*dt

h = 2*dt;      %% Runge-Kutta
errth = 5e-4;  %% Runge-Kutta
beta = 1;      %% Runge-Kutta
c = 1;         %% Runge-Kutta
%% 家览丁把计 ㎝ 繦诀把计 END %%

%% FUZZY interpolation %%
pv1 = [-20 0 20];  %% phi
pv2 = [-20 0 20];  %% theta 
pv3 = [-20 0 20];  %% psi
FindLocalMatrix();
%% FUZZY interpolation END %%

%% LMI find K %%
rho=20;
%Q = eye(12);
Q = zeros(24,12);
Q(1:12,:) = 0.01*diag([1,0.001,1,0.001,1,0.001,1,0.001,1,0.001,1,0.001]);
Q(13:24,:) = 0.01*diag([1,0.001,1,0.001,1,0.001,1,0.001,1,0.001,1,0.001]);
LMI_find_K_multi(rho,Q);
%%  LMI find K END %%

%% check system stability %%
check_stability();
%% check system stability END %%

%% 繦诀把计 %% 
dW = sqrt(dt)*randn(1,N);   %% for weiner
W = cumsum(dW);            %% for weiner
Count = 50;                    %% for poisson
lamda = 0.5;                   %% for poisson
Times = Poisson(lamda,Count);  %% for poisson
%% 倒 initial value %%
Xt_L = [0.3 0 0.8 0 0.3 0 0 0 0 0 0 0]';
Xt_F1 = [0.3 0 0.8 0 0.3 0 0 0 0 0 0 0]';
Xt = [Xt_L;Xt_F1];
index = [1 1 1 1];
Xt = [Xt zeros(length(Xt),L-1)];
r = zeros(12,L);
d = zeros(4,L);
n1=0;n2=1;n3=2;n4=3;n5=4;

for j = 1:L
    Winc = sum(dW(R*(j-1)+1:R*j));
    %% DE
    [k1 r1 r2 d1] = multi_UAV_DE_NL(j*h,Xt(:,j),index);
    [k2 r0 r0 d2] = multi_UAV_DE_NL(j*h+h/2,Xt(:,j)+h*k1/2,index);
    [k3 r0 r0 d3] = multi_UAV_DE_NL(j*h+h/2,Xt(:,j)+h*k2/2,index);
    [k4 r0 r0 d4] = multi_UAV_DE_NL(j*h+h,Xt(:,j)+h*k3,index);
    Xt(:,j+1)=Xt(:,j)+h*(k1+2*k2+2*k3+k4)/6;
    if j*h<=Times(c)+2*h && j*h>Times(c)
        Xt(:,j+1)=Xt(:,j+1)+[0;Xt(2,j+1);0;Xt(4,j+1);0;Xt(6,j+1);0;Xt(8,j+1);0;Xt(10,j+1);0;Xt(12,j+1);zeros(12,1)]*Winc...
                         +[0;Xt(2,j+1);0;Xt(4,j+1);0;Xt(6,j+1);0;Xt(8,j+1);0;Xt(10,j+1);0;Xt(12,j+1);zeros(12,1)]*1/10;
        c = c+1;
    else
        Xt(:,j+1)=Xt(:,j+1)+[0;Xt(2,j+1);0;Xt(4,j+1);0;Xt(6,j+1);0;Xt(8,j+1);0;Xt(10,j+1);0;Xt(12,j+1);zeros(12,1)]*Winc;
    end
     
    r1(:,j+1) = r1;
    r2(:,j+1) = r2;
    d(:,j+1) = d1;
    
    %meam_square_error = sum(Xt(3,:).^2);
    %meam_square_error1 = meam_square_error;
end

dp = 1; ep = 1; level = 0;

for j = 0:L
   if j*h<=Times(dp)+1.5*h && j*h>Times(dp)
       level = level + 1;
       Counting(ep) = level;
       ep = ep + 1;
       dp = dp + 1;
   else
       Counting(ep) = level;
       ep = ep + 1;
   end
end

figure(1)
subplot(3,2,1)
plot(0:T/L:T,Xt(1,:),'r')
hold on
plot(0:T/L:T,r1(1,:),'black')
legend('x(t)','r_ x(t)')
title('leader x')
subplot(3,2,3)
plot(0:T/L:T,Xt(3,:),'r')
hold on
plot(0:T/L:T,r1(3,:),'black')
legend('1y(t)','r_ y(t)')
title('leader y')
subplot(3,2,5)
plot(0:T/L:T,Xt(5,:),'r')
hold on
plot(0:T/L:T,r1(5,:),'black')
legend('z(t)','r_ z(t)')
title('leader z')
subplot(3,2,2)
plot(0:T/L:T,Xt(2,:),'r')
hold on
plot(0:T/L:T,r1(2,:),'black')
legend('Vx(t)','r_ Vx(t)')
title('leader Vx')
subplot(3,2,4)
plot(0:T/L:T,Xt(4,:),'r')
hold on
plot(0:T/L:T,r1(4,:),'black')
legend('Vy(t)','r_ Vy(t)')
title('leader Vy')
subplot(3,2,6)
plot(0:T/L:T,Xt(6,:),'r')
hold on
plot(0:T/L:T,r1(6,:),'black')
legend('Vz(t)','r_ Vz(t)')
title('leader Vz')

figure(2)
subplot(3,2,1)
plot(0:T/L:T,Xt(7,:),'r')
hold on
plot(0:T/L:T,r1(7,:),'black')
legend('phi(t)','r_ phi(t)')
title('leader phi')
subplot(3,2,3)
plot(0:T/L:T,Xt(9,:),'r')
hold on
plot(0:T/L:T,r1(9,:),'black')
legend('theta(t)','r_ theta(t)')
title('leader theta')
subplot(3,2,5)
plot(0:T/L:T,Xt(11,:),'r')
hold on
plot(0:T/L:T,r1(11,:),'black')
legend('pshi(t)','r_ psi(t)')
title('leader psi')
subplot(3,2,2)
plot(0:T/L:T,Xt(8,:),'r')
hold on
plot(0:T/L:T,r1(8,:),'black')
legend('Vphi(t)','Vr_ phi(t)')
title('leader Vphi')
subplot(3,2,4)
plot(0:T/L:T,Xt(10,:),'r')
hold on
plot(0:T/L:T,r1(10,:),'black')
legend('Vtheta(t)','r_ Vtheta(t)')
title('leader Vtheta')
subplot(3,2,6)
plot(0:T/L:T,Xt(12,:),'r')
hold on
plot(0:T/L:T,r1(12,:),'black')
legend('Vpshi(t)','r_ Vpsi(t)')
title('leader Vpsi')

figure(3)
subplot(3,2,1)
plot(0:T/L:T,Xt(1+12,:),'r')
hold on
plot(0:T/L:T,r2(1,:),'black')
legend('x(t)','r_ x(t)')
title('follower x')
subplot(3,2,3)
plot(0:T/L:T,Xt(3+12,:),'r')
hold on
plot(0:T/L:T,r2(3,:),'black')
legend('1y(t)','r_ y(t)')
title('follower y')
subplot(3,2,5)
plot(0:T/L:T,Xt(5+12,:),'r')
hold on
plot(0:T/L:T,r2(5,:),'black')
legend('z(t)','r_ z(t)')
title('follower z')
subplot(3,2,2)
plot(0:T/L:T,Xt(2+12,:),'r')
hold on
plot(0:T/L:T,r2(2,:),'black')
legend('Vx(t)','r_ Vx(t)')
title('follower Vx')
subplot(3,2,4)
plot(0:T/L:T,Xt(4+12,:),'r')
hold on
plot(0:T/L:T,r2(4,:),'black')
legend('Vy(t)','r_ Vy(t)')
title('follower Vy')
subplot(3,2,6)
plot(0:T/L:T,Xt(6+12,:),'r')
hold on
plot(0:T/L:T,r2(6,:),'black')
legend('Vz(t)','r_ Vz(t)')
title('follower Vz')

figure(4)
subplot(3,2,1)
plot(0:T/L:T,Xt(7+12,:),'r')
hold on
plot(0:T/L:T,r2(7,:),'black')
legend('x(t)','r_ x(t)')
title('follower x')
subplot(3,2,3)
plot(0:T/L:T,Xt(9+12,:),'r')
hold on
plot(0:T/L:T,r2(9,:),'black')
legend('1y(t)','r_ y(t)')
title('follower y')
subplot(3,2,5)
plot(0:T/L:T,Xt(11+12,:),'r')
hold on
plot(0:T/L:T,r2(11,:),'black')
legend('z(t)','r_ z(t)')
title('follower z')
subplot(3,2,2)
plot(0:T/L:T,Xt(8+12,:),'r')
hold on
plot(0:T/L:T,r2(8,:),'black')
legend('Vx(t)','r_ Vx(t)')
title('follower Vx')
subplot(3,2,4)
plot(0:T/L:T,Xt(10+12,:),'r')
hold on
plot(0:T/L:T,r2(10,:),'black')
legend('Vy(t)','r_ Vy(t)')
title('follower Vy')
subplot(3,2,6)
plot(0:T/L:T,Xt(12+12,:),'r')
hold on
plot(0:T/L:T,r2(12,:),'black')
legend('Vz(t)','r_ Vz(t)')
title('follower Vz')

figure(7)
plot(0:T/L:T,r(1,:),'r')
hold on
plot(0:T/L:T,r(3,:),'g')
legend('r_ phi(t)','r_ theta(t)')
plot(0:T/L:T,r(7,:),'b')
hold on
plot(0:T/L:T,r(9,:),'y')
legend('xd(t)','yd(t)','r_ phi(t)','r_ theta(t)')


figure(8)
plot(0:T/L:T,d(1,:),'r')
hold on
plot(0:T/L:T,d(3,:),'g')
legend('d1','d2')


figure(9)
plot(0:T/L:T,Counting,'b')
xlabel('Time')
ylabel('p(t)')
title('Poisson counting process')





