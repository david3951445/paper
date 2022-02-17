% Runge-Kutta-Fehlberg method on nonlinear stochastic ONE UAV tracking model
clc; clear all; close all;
global Kx Ky Kz Kphi Ktheta Kpsi Jphi Jtheta Jpsi m g Ar mar_stable num
global pv1 pv2 pv3 Local
load multi_UAV.mat;

%% 家览丁把计 ㎝ 繦诀把计 %%
dt = 1e-3;T = 10; N = T/dt; 
R = 2; Dt = R*dt; L = N/R; % L steps size Dt = R*dt

h = 2*dt;      %% Runge-Kutta
errth = 5e-4;  %% Runge-Kutta
beta = 1;      %% Runge-Kutta
c = 1;         %% Runge-Kutta

%% 繦诀把计 %% 
dW1 = sqrt(dt)*randn(1,N);      %% for weiner 1
W1 = cumsum(dW1);                %% for weiner 1
dW2 = sqrt(dt)*randn(1,N);      %% for weiner 2
W2 = cumsum(dW2);                %% for weiner 2
dW3 = sqrt(dt)*randn(1,N);      %% for weiner 3
W3 = cumsum(dW3);                %% for weiner 3
dW4 = sqrt(dt)*randn(1,N);      %% for weiner 4
W4 = cumsum(dW4);                %% for weiner 4
dW5 = sqrt(dt)*randn(1,N);      %% for weiner 5
W5 = cumsum(dW5);                %% for weiner 5
Count = 50;                     %% for poisson
lamda = 0.5;                    %% for poisson
Times1 = Poisson(lamda,Count);  %% for poisson 1
c1 = 1;                         %% for poisson 1
Times2 = Poisson(lamda,Count);  %% for poisson 2
c2 = 1;                         %% for poisson 2
Times3 = Poisson(lamda,Count);  %% for poisson 3
c3 = 1;                         %% for poisson 3
Times4 = Poisson(lamda,Count);  %% for poisson 4
c4 = 1;                         %% for poisson 4
Times5 = Poisson(lamda,Count);  %% for poisson 5
c5 = 1;                         %% for poisson 5


%% 倒 initial value %%
index = [1 1 1 1];
Xt_L = [0.3 0 0.8 0 0.3 0 0 0 0 0 0 0]';
Xt_F1 = [0.3 0 0.8 0 0.3 0 0 0 0 0 0 0]';
Xt_F2 = [0.3 0 0.8 0 0.3 0 0 0 0 0 0 0]';
Xt_F3 = [0.3 0 0.8 0 0.3 0 0 0 0 0 0 0]';
Xt_F4 = [0.3 0 0.8 0 0.3 0 0 0 0 0 0 0]';
Xt1 = [Xt_L;Xt_F1];
Xt2 = [Xt_L;Xt_F2];
Xt3 = [Xt_L;Xt_F3];
Xt4 = [Xt_L;Xt_F4];
Xt1 = [Xt1 zeros(length(Xt1),L-1)];
Xt2 = [Xt2 zeros(length(Xt2),L-1)];
Xt3 = [Xt3 zeros(length(Xt3),L-1)];
Xt4 = [Xt4 zeros(length(Xt4),L-1)];
r1 = zeros(12,L);
r2 = zeros(12,L);
d = zeros(4,L);
n1=0;n2=1;n3=2;n4=3;n5=4;

%% Leader + Follower 1 DE
for j = 1:L
    Winc1 = sum(dW1(R*(j-1)+1:R*j));
    Winc2 = sum(dW2(R*(j-1)+1:R*j));

    [k1 kr1 kr2 d1] = multi_UAV_DE_1(j*h,Xt1(:,j),index);
    [k2 kr0 kr0 d2] = multi_UAV_DE_1(j*h+h/2,Xt1(:,j)+h*k1/2,index);
    [k3 kr0 kr0 d3] = multi_UAV_DE_1(j*h+h/2,Xt1(:,j)+h*k2/2,index);
    [k4 kr0 kr0 d4] = multi_UAV_DE_1(j*h+h,Xt1(:,j)+h*k3,index);
    Xt1(:,j+1)=Xt1(:,j)+h*(k1+2*k2+2*k3+k4)/6;
      %% weiner
    Xt1(:,j+1)=Xt1(:,j+1)+[0;Xt1(2,j+1);0;Xt1(4,j+1);0;Xt1(6,j+1);0;Xt1(8,j+1);0;Xt1(10,j+1);0;Xt1(12,j+1);zeros(12,1)]*(Winc1/10)...
                         +[zeros(12,1);0;Xt1(2,j+1);0;Xt1(4,j+1);0;Xt1(6,j+1);0;Xt1(8,j+1);0;Xt1(10,j+1);0;Xt1(12,j+1)]*(Winc2/10);    
      %% poisson
    if j*h<=Times1(c1)+2*h && j*h>Times1(c1)
        Xt1(:,j+1)=Xt1(:,j+1)+[0;Xt1(2,j+1);0;Xt1(4,j+1);0;Xt1(6,j+1);0;Xt1(8,j+1);0;Xt1(10,j+1);0;Xt1(12,j+1);zeros(12,1)]*1;
        c1 = c1+1;
    else
        Xt1(:,j+1)=Xt1(:,j+1);
    end
    if j*h<=Times2(c2)+2*h && j*h>Times2(c2)
        Xt1(:,j+1)=Xt1(:,j+1)+[zeros(12,1);0;Xt1(2,j+1);0;Xt1(4,j+1);0;Xt1(6,j+1);0;Xt1(8,j+1);0;Xt1(10,j+1);0;Xt1(12,j+1)]*1;
        c2 = c2+1;
    else
        Xt1(:,j+1)=Xt1(:,j+1);
    end
     
    r1(:,j+1) = kr1;
    r2(:,j+1) = kr2;
    d(:,j+1) = d1;
end

%% Leader + Follower 2 DE
for j = 1:L
    Winc1 = sum(dW1(R*(j-1)+1:R*j));
    Winc3 = sum(dW3(R*(j-1)+1:R*j));

    [k1 kr1 kr2 d1] = multi_UAV_DE_2(j*h,Xt2(:,j),index);
    [k2 kr0 kr0 d2] = multi_UAV_DE_2(j*h+h/2,Xt2(:,j)+h*k1/2,index);
    [k3 kr0 kr0 d3] = multi_UAV_DE_2(j*h+h/2,Xt2(:,j)+h*k2/2,index);
    [k4 kr0 kr0 d4] = multi_UAV_DE_2(j*h+h,Xt2(:,j)+h*k3,index);
    Xt2(:,j+1)=Xt2(:,j)+h*(k1+2*k2+2*k3+k4)/6;
      %% weiner
    Xt2(:,j+1)=Xt2(:,j+1)+[0;Xt2(2,j+1);0;Xt2(4,j+1);0;Xt2(6,j+1);0;Xt2(8,j+1);0;Xt2(10,j+1);0;Xt2(12,j+1);zeros(12,1)]*(Winc1/10)...
                         +[zeros(12,1);0;Xt2(2,j+1);0;Xt2(4,j+1);0;Xt2(6,j+1);0;Xt2(8,j+1);0;Xt2(10,j+1);0;Xt2(12,j+1)]*(Winc3/10);    
      %% poisson
    if j*h<=Times1(c1)+2*h && j*h>Times1(c1)
        Xt2(:,j+1)=Xt2(:,j+1)+[0;Xt2(2,j+1);0;Xt2(4,j+1);0;Xt2(6,j+1);0;Xt2(8,j+1);0;Xt2(10,j+1);0;Xt2(12,j+1);zeros(12,1)]*1;
        c1 = c1+1;
    else
        Xt2(:,j+1)=Xt2(:,j+1);
    end
    if j*h<=Times3(c3)+2*h && j*h>Times3(c3)
        Xt2(:,j+1)=Xt2(:,j+1)+[zeros(12,1);0;Xt2(2,j+1);0;Xt2(4,j+1);0;Xt2(6,j+1);0;Xt2(8,j+1);0;Xt2(10,j+1);0;Xt2(12,j+1)]*1;
        c3 = c3+1;
    else
        Xt2(:,j+1)=Xt2(:,j+1);
    end     
end

%% Leader + Follower 3 DE
for j = 1:L
    Winc1 = sum(dW1(R*(j-1)+1:R*j));
    Winc4 = sum(dW4(R*(j-1)+1:R*j));

    [k1 kr1 kr2 d1] = multi_UAV_DE_3(j*h,Xt3(:,j),index);
    [k2 kr0 kr0 d2] = multi_UAV_DE_3(j*h+h/2,Xt3(:,j)+h*k1/2,index);
    [k3 kr0 kr0 d3] = multi_UAV_DE_3(j*h+h/2,Xt3(:,j)+h*k2/2,index);
    [k4 kr0 kr0 d4] = multi_UAV_DE_3(j*h+h,Xt3(:,j)+h*k3,index);
    Xt3(:,j+1)=Xt3(:,j)+h*(k1+2*k2+2*k3+k4)/6;
      %% weiner
    Xt3(:,j+1)=Xt3(:,j+1)+[0;Xt3(2,j+1);0;Xt3(4,j+1);0;Xt3(6,j+1);0;Xt3(8,j+1);0;Xt3(10,j+1);0;Xt3(12,j+1);zeros(12,1)]*(Winc1/10)...
                         +[zeros(12,1);0;Xt3(2,j+1);0;Xt3(4,j+1);0;Xt3(6,j+1);0;Xt3(8,j+1);0;Xt3(10,j+1);0;Xt3(12,j+1)]*(Winc4/10);    
      %% poisson
    if j*h<=Times1(c1)+2*h && j*h>Times1(c1)
        Xt3(:,j+1)=Xt3(:,j+1)+[0;Xt3(2,j+1);0;Xt3(4,j+1);0;Xt3(6,j+1);0;Xt3(8,j+1);0;Xt3(10,j+1);0;Xt3(12,j+1);zeros(12,1)]*1;
        c1 = c1+1;
    else
        Xt3(:,j+1)=Xt3(:,j+1);
    end
    if j*h<=Times4(c4)+2*h && j*h>Times4(c4)
        Xt3(:,j+1)=Xt3(:,j+1)+[zeros(12,1);0;Xt3(2,j+1);0;Xt3(4,j+1);0;Xt3(6,j+1);0;Xt3(8,j+1);0;Xt3(10,j+1);0;Xt3(12,j+1)]*1;
        c4 = c4+1;
    else
        Xt3(:,j+1)=Xt3(:,j+1);
    end     
end
%% Leader + Follower 4 DE
for j = 1:L
    Winc1 = sum(dW1(R*(j-1)+1:R*j));
    Winc5 = sum(dW5(R*(j-1)+1:R*j));

    [k1 kr1 kr2 d1] = multi_UAV_DE_4(j*h,Xt4(:,j),index);
    [k2 kr0 kr0 d2] = multi_UAV_DE_4(j*h+h/2,Xt4(:,j)+h*k1/2,index);
    [k3 kr0 kr0 d3] = multi_UAV_DE_4(j*h+h/2,Xt4(:,j)+h*k2/2,index);
    [k4 kr0 kr0 d4] = multi_UAV_DE_4(j*h+h,Xt4(:,j)+h*k3,index);
    Xt4(:,j+1)=Xt4(:,j)+h*(k1+2*k2+2*k3+k4)/6;
      %% weiner
    Xt4(:,j+1)=Xt4(:,j+1)+[0;Xt4(2,j+1);0;Xt4(4,j+1);0;Xt4(6,j+1);0;Xt4(8,j+1);0;Xt4(10,j+1);0;Xt4(12,j+1);zeros(12,1)]*(Winc1/10)...
                         +[zeros(12,1);0;Xt4(2,j+1);0;Xt4(4,j+1);0;Xt4(6,j+1);0;Xt4(8,j+1);0;Xt4(10,j+1);0;Xt4(12,j+1)]*(Winc5/10);    
      %% poisson
    if j*h<=Times1(c1)+2*h && j*h>Times1(c1)
        Xt4(:,j+1)=Xt4(:,j+1)+[0;Xt4(2,j+1);0;Xt4(4,j+1);0;Xt4(6,j+1);0;Xt4(8,j+1);0;Xt4(10,j+1);0;Xt4(12,j+1);zeros(12,1)]*1;
        c1 = c1+1;
    else
        Xt4(:,j+1)=Xt4(:,j+1);
    end
    if j*h<=Times5(c5)+2*h && j*h>Times5(c5)
        Xt4(:,j+1)=Xt4(:,j+1)+[zeros(12,1);0;Xt4(2,j+1);0;Xt4(4,j+1);0;Xt4(6,j+1);0;Xt4(8,j+1);0;Xt4(10,j+1);0;Xt4(12,j+1)]*1;
        c5 = c5+1;
    else
        Xt4(:,j+1)=Xt4(:,j+1);
    end     
end

figure(1)
subplot(3,2,1)
plot(0:T/L:T,Xt1(1,:),'r')
hold on
plot(0:T/L:T,r1(1,:),'black')
legend('leader x(t)','r_ x(t)')
title('leader x')
subplot(3,2,3)
plot(0:T/L:T,Xt1(3,:),'r')
hold on
plot(0:T/L:T,r1(3,:),'black')
legend('leader y(t)','r_ y(t)')
title('leader y')
subplot(3,2,5)
plot(0:T/L:T,Xt1(5,:),'r')
hold on
plot(0:T/L:T,r1(5,:),'black')
legend('leader z(t)','r_ z(t)')
title('leader z')
subplot(3,2,2)
plot(0:T/L:T,Xt1(2,:),'r')
hold on
plot(0:T/L:T,r1(2,:),'black')
legend('leader Vx(t)','r_ Vx(t)')
title('leader Vx')
subplot(3,2,4)
plot(0:T/L:T,Xt1(4,:),'r')
hold on
plot(0:T/L:T,r1(4,:),'black')
legend('leader Vy(t)','r_ Vy(t)')
title('leader Vy')
subplot(3,2,6)
plot(0:T/L:T,Xt1(6,:),'r')
hold on
plot(0:T/L:T,r1(6,:),'black')
legend('leader Vz(t)','r_ Vz(t)')
title('leader Vz')

figure(2)
subplot(3,2,1)
plot(0:T/L:T,Xt1(7,:),'r')
hold on
plot(0:T/L:T,r1(7,:),'black')
legend('leader phi(t)','r_ phi(t)')
title('leader phi')
subplot(3,2,3)
plot(0:T/L:T,Xt1(9,:),'r')
hold on
plot(0:T/L:T,r1(9,:),'black')
legend('leader theta(t)','r_ theta(t)')
title('leader theta')
subplot(3,2,5)
plot(0:T/L:T,Xt1(11,:),'r')
hold on
plot(0:T/L:T,r1(11,:),'black')
legend('leader psi(t)','r_ psi(t)')
title('leader psi')
subplot(3,2,2)
plot(0:T/L:T,Xt1(8,:),'r')
hold on
plot(0:T/L:T,r1(8,:),'black')
legend('leader Vphi(t)','Vr_ phi(t)')
title('leader Vphi')
subplot(3,2,4)
plot(0:T/L:T,Xt1(10,:),'r')
hold on
plot(0:T/L:T,r1(10,:),'black')
legend('leader Vtheta(t)','r_ Vtheta(t)')
title('leader Vtheta')
subplot(3,2,6)
plot(0:T/L:T,Xt1(12,:),'r')
hold on
plot(0:T/L:T,r1(12,:),'black')
legend('leader Vpshi(t)','r_ Vpsi(t)')
title('leader Vpsi')

figure(3)
subplot(3,2,1)
plot(0:T/L:T,Xt1(1+12,:),'r')
hold on
plot(0:T/L:T,r2(1,:),'black')
legend('follower x(t)','leader x(t)')
title('follower x')
subplot(3,2,3)
plot(0:T/L:T,Xt1(3+12,:),'r')
hold on
plot(0:T/L:T,r2(3,:),'black')
legend('follower y(t)','leader y(t)')
title('follower y')
subplot(3,2,5)
plot(0:T/L:T,Xt1(5+12,:),'r')
hold on
plot(0:T/L:T,r2(5,:),'black')
legend('follower z(t)','leader z(t)')
title('follower z')
subplot(3,2,2)
plot(0:T/L:T,Xt1(2+12,:),'r')
hold on
plot(0:T/L:T,r2(2,:),'black')
legend('follower Vx(t)','leader Vx(t)')
title('follower Vx')
subplot(3,2,4)
plot(0:T/L:T,Xt1(4+12,:),'r')
hold on
plot(0:T/L:T,r2(4,:),'black')
legend('follower Vy(t)','leader Vy(t)')
title('follower Vy')
subplot(3,2,6)
plot(0:T/L:T,Xt1(6+12,:),'r')
hold on
plot(0:T/L:T,r2(6,:),'black')
legend('follower Vz(t)','leader Vz(t)')
title('follower Vz')

figure(4)
subplot(3,2,1)
plot(0:T/L:T,Xt1(7+12,:),'r')
hold on
plot(0:T/L:T,r2(7,:),'black')
legend('follower phi(t)','leader phi(t)')
title('follower phi')
subplot(3,2,3)
plot(0:T/L:T,Xt1(9+12,:),'r')
hold on
plot(0:T/L:T,r2(9,:),'black')
legend('follower theta(t)','leader theta(t)')
title('follower theta')
subplot(3,2,5)
plot(0:T/L:T,Xt1(11+12,:),'r')
hold on
plot(0:T/L:T,r2(11,:),'black')
legend('follower psi(t)','leader psi(t)')
title('follower psi')
subplot(3,2,2)
plot(0:T/L:T,Xt1(8+12,:),'r')
hold on
plot(0:T/L:T,r2(8,:),'black')
legend('follower Vphi(t)','leader Vphi(t)')
title('follower Vphi')
subplot(3,2,4)
plot(0:T/L:T,Xt1(10+12,:),'r')
hold on
plot(0:T/L:T,r2(10,:),'black')
legend('follower Vtheta(t)','leader Vtheta(t)')
title('follower Vtheta')
subplot(3,2,6)
plot(0:T/L:T,Xt1(12+12,:),'r')
hold on
plot(0:T/L:T,r2(12,:),'black')
legend('follower Vpsi(t)','leader Vpsi(t)')
title('follower Vpsi')

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





