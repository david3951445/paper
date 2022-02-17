% Runge-Kutta-Fehlberg method on nonlinear stochastic ONE UAV tracking model
clc; clear all; close all;
global Kx Ky Kz Kphi Ktheta Kpsi Jphi Jtheta Jpsi m g Ar mar_stable num
global pv1 pv2 pv3 Local
load multi_UAV.mat;
load stocastic_item.mat;

Xt2 = [Xt_L;Xt_F2];
Xt2 = [Xt2 zeros(length(Xt2),L-1)];
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
                         +[zeros(12,1);0;Xt2(14,j+1);0;Xt2(16,j+1);0;Xt2(18,j+1);0;Xt2(20,j+1);0;Xt2(22,j+1);0;Xt2(24,j+1)]*(Winc3/10);    
      %% poisson
    if j*h<=Times1(c1)+2*h && j*h>Times1(c1)
        Xt2(:,j+1)=Xt2(:,j+1)+[0;Xt2(2,j+1);0;Xt2(4,j+1);0;Xt2(6,j+1);0;Xt2(8,j+1);0;Xt2(10,j+1);0;Xt2(12,j+1);zeros(12,1)]*1/2;
        c1 = c1+1;
    else
        Xt2(:,j+1)=Xt2(:,j+1);
    end
    if j*h<=Times3(c3)+2*h && j*h>Times3(c3)
        Xt2(:,j+1)=Xt2(:,j+1)+[zeros(12,1);0;Xt2(14,j+1);0;Xt2(16,j+1);0;Xt2(18,j+1);0;Xt2(20,j+1);0;Xt2(22,j+1);0;Xt2(24,j+1)]*1/2;
        c3 = c3+1;
    else
        Xt2(:,j+1)=Xt2(:,j+1);
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
   if j*h<=Times3(dp3)+1.5*h && j*h>Times3(dp3)
       level3 = level3 + 1;
       Counting3(ep3) = level3;
       ep3 = ep3 + 1;
       dp3 = dp3 + 1;
   else
       Counting3(ep3) = level3;
       ep3 = ep3 + 1;
   end
end


save UAV2.mat;
