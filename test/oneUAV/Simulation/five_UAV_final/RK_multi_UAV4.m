% Runge-Kutta-Fehlberg method on nonlinear stochastic ONE UAV tracking model
clc; clear all; close all;
global Kx Ky Kz Kphi Ktheta Kpsi Jphi Jtheta Jpsi m g Ar mar_stable num
global pv1 pv2 pv3 Local
load multi_UAV.mat;
load stocastic_item.mat;

Xt4 = [Xt_L;Xt_F4];
Xt4 = [Xt4 zeros(length(Xt4),L-1)];
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
                         +[zeros(12,1);0;Xt4(14,j+1);0;Xt4(16,j+1);0;Xt4(18,j+1);0;Xt4(20,j+1);0;Xt4(22,j+1);0;Xt4(24,j+1)]*(Winc5/10);    
      %% poisson
    if j*h<=Times1(c1)+2*h && j*h>Times1(c1)
        Xt4(:,j+1)=Xt4(:,j+1)+[0;Xt4(2,j+1);0;Xt4(4,j+1);0;Xt4(6,j+1);0;Xt4(8,j+1);0;Xt4(10,j+1);0;Xt4(12,j+1);zeros(12,1)]*1/2;
        c1 = c1+1;
    else
        Xt4(:,j+1)=Xt4(:,j+1);
    end
    if j*h<=Times5(c5)+2*h && j*h>Times5(c5)
        Xt4(:,j+1)=Xt4(:,j+1)+[zeros(12,1);0;Xt4(14,j+1);0;Xt4(16,j+1);0;Xt4(18,j+1);0;Xt4(20,j+1);0;Xt4(22,j+1);0;Xt4(24,j+1)]*1/2;
        c5 = c5+1;
    else
        Xt4(:,j+1)=Xt4(:,j+1);
    end     
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
   if j*h<=Times5(dp5)+1.5*h && j*h>Times5(dp5)
       level5 = level5 + 1;
       Counting5(ep5) = level5;
       ep5 = ep5 + 1;
       dp5 = dp5 + 1;
   else
       Counting5(ep5) = level5;
       ep5 = ep5 + 1;
   end
end

save UAV4.mat;






