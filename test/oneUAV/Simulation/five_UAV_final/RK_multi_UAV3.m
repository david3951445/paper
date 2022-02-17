% Runge-Kutta-Fehlberg method on nonlinear stochastic ONE UAV tracking model
clc; clear all; close all;
global Kx Ky Kz Kphi Ktheta Kpsi Jphi Jtheta Jpsi m g Ar mar_stable num
global pv1 pv2 pv3 Local
load multi_UAV.mat;
load stocastic_item.mat;

Xt3 = [Xt_L;Xt_F3];
Xt3 = [Xt3 zeros(length(Xt3),L-1)];
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
                         +[zeros(12,1);0;Xt3(14,j+1);0;Xt3(16,j+1);0;Xt3(18,j+1);0;Xt3(20,j+1);0;Xt3(22,j+1);0;Xt3(24,j+1)]*(Winc4/10);    
      %% poisson
    if j*h<=Times1(c1)+2*h && j*h>Times1(c1)
        Xt3(:,j+1)=Xt3(:,j+1)+[0;Xt3(2,j+1);0;Xt3(4,j+1);0;Xt3(6,j+1);0;Xt3(8,j+1);0;Xt3(10,j+1);0;Xt3(12,j+1);zeros(12,1)]*1/2;
        c1 = c1+1;
    else
        Xt3(:,j+1)=Xt3(:,j+1);
    end
    if j*h<=Times4(c4)+2*h && j*h>Times4(c4)
        Xt3(:,j+1)=Xt3(:,j+1)+[zeros(12,1);0;Xt3(14,j+1);0;Xt3(16,j+1);0;Xt3(18,j+1);0;Xt3(20,j+1);0;Xt3(22,j+1);0;Xt3(24,j+1)]*1/2;
        c4 = c4+1;
    else
        Xt3(:,j+1)=Xt3(:,j+1);
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
   if j*h<=Times4(dp4)+1.5*h && j*h>Times4(dp4)
       level4 = level4 + 1;
       Counting4(ep4) = level4;
       ep4 = ep4 + 1;
       dp4 = dp4 + 1;
   else
       Counting4(ep4) = level4;
       ep4 = ep4 + 1;
   end
end

save UAV3.mat;
