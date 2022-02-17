% Runge-Kutta-Fehlberg method on nonlinear stochastic ONE UAV tracking model
clc; clear all; close all;
global Kx Ky Kz Kphi Ktheta Kpsi Jphi Jtheta Jpsi m g Ar mar_stable num
global pv1 pv2 pv3 Local
load multi_UAV.mat;
load stocastic_item.mat;

Xt1 = [Xt_L;Xt_F1];
Xt1 = [Xt1 zeros(length(Xt1),L-1)];
r1 = zeros(12,L);
r2 = zeros(12,L);
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
                         +[zeros(12,1);0;Xt1(14,j+1);0;Xt1(16,j+1);0;Xt1(18,j+1);0;Xt1(20,j+1);0;Xt1(22,j+1);0;Xt1(24,j+1)]*(Winc2/10);    
      %% poisson
    if j*h<=Times1(c1)+2*h && j*h>Times1(c1)
        Xt1(:,j+1)=Xt1(:,j+1)+[0;Xt1(2,j+1);0;Xt1(4,j+1);0;Xt1(6,j+1);0;Xt1(8,j+1);0;Xt1(10,j+1);0;Xt1(12,j+1);zeros(12,1)]*1/2;
        c1 = c1+1;
    else
        Xt1(:,j+1)=Xt1(:,j+1);
    end
    if j*h<=Times2(c2)+2*h && j*h>Times2(c2)
        Xt1(:,j+1)=Xt1(:,j+1)+[zeros(12,1);0;Xt1(14,j+1);0;Xt1(16,j+1);0;Xt1(18,j+1);0;Xt1(20,j+1);0;Xt1(22,j+1);0;Xt1(24,j+1)]*1/2;
        c2 = c2+1;
    else
        Xt1(:,j+1)=Xt1(:,j+1);
    end
     
    r1(:,j+1) = kr1;
    r2(:,j+1) = kr2;
    d(:,j+1) = d1;
    j1=j
end
for j = 0:L
   if j*h<=Times1(dp1)+1.5*h && j*h>Times1(dp1)
       level1 = level1 + 1;
       Counting1(ep1) = level1;
       ep1 = ep1 + 1;
       dp1 = dp1 + 1;
   else
       Counting1(ep1) = level1;
       ep1 = ep1 + 1;
   end
   if j*h<=Times2(dp2)+1.5*h && j*h>Times2(dp2)
       level2 = level2 + 1;
       Counting2(ep2) = level2;
       ep2 = ep2 + 1;
       dp2 = dp2 + 1;
   else
       Counting2(ep2) = level2;
       ep2 = ep2 + 1;
   end
end
save UAV1.mat;
