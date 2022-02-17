
clear all;
close all;
clc

%% mosek setting
%addpath(genpath('C:\Program Files\Mosek\9.2\toolbox\R2015a')) % school
%addpath(genpath('E:\matlabfolder\mosek\9.2\toolbox\R2015a')) % home
addpath(genpath('C:\Program Files\Mosek\9.2\toolbox\R2015a')) % home

options = sdpsettings('solver','mosek');
options = sdpsettings(options,'verbose',0);

%%參數設計
%% parameter
global  Kx Ky Kz Kphi Ktheta Kpsi Jx Jy Jz m g
    Kx = 0.01;
    Ky = 0.01;
    Kz = 0.01;
    Kphi = 0.013;
    Ktheta = 0.013;
    Kpsi = 0.013;
    Jx = 1.25;
    Jy = 1.25;
    Jz = 2.2;
    m = 3;
    g = 9.81;
%%fuzzy rule local system
z2=[0.499 g];%%z2)
phi1=[-0.05 0.05];%%phi1
phi2= [-0.015 0.015];%%phi2
theta1= [-0.05 0.05];%%theta1
theta2= [-0.015 0.015];%%theta2
psi1=pi/2;%equilibrium point
psi2=0;%equilibrium point

%%local system
index=length(z2)*length(phi1)*length(phi2)*length(theta1)*length(theta2)*length(psi1)*length(psi2);
sum=1;


for i=1:length(z2)
    for j=1:length(phi1)
        for k=1:length(phi2)
             for l=1:length(theta1)
                  for n=1:length(theta2)
   A(:,:,sum) = [0 0 0 0 0 0 1 0 0 0 0 0; 
                 0 0 0 0 0 0 0 1 0 0 0 0;
                 0 0 0 0 0 0 0 0 1 0 0 0;
                 0 0 0 0 0 0 0 0 0 1 0 0;
                 0 0 0 0 0 0 0 0 0 0 1 0;
                 0 0 0 0 0 0 0 0 0 0 0 1;
                 0 0 0 0 0 0 -Kx/m 0 0 0 0 0;
                 0 0 0 0 0 0 0 -Ky/m 0 0 0 0;
                 0 0 0 0 0 0 0 0 -Kz/m-g/z2(i) 0 0 0;
                 0 0 0 0 0.5*(Jy-Jz)/Jx*psi1 0.5*(Jy-Jz)/Jx*theta1(l) 0 0 0 -Kphi/Jy 0 0;
                 0 0 0 0.5*(Jz-Jx)/Jy*psi1 0 0.5*(Jz-Jx)/Jy*phi1(i) 0 0 0 0 -Ktheta/Jy 0;
                 0 0 0 0.5*(Jy-Jx)/Jz*theta1(l) 0.5*(Jz-Jx)/Jy*phi1(j) 0 0 0 0 0 0 -Kpsi/Jz];
            
    
  B(:,:,sum)=  [0 0 0 0;
                0 0 0 0;
                0 0 0 0;
                0 0 0 0;
                0 0 0 0;
                0 0 0 0;
               (cos(phi1(j))*sin(theta1(l))*cos(psi1)+sin(phi1(j))*sin(psi1))/m 0 0 0;
               (cos(phi1(j))*sin(theta1(l))*sin(psi1)-sin(phi1(j))*cos(psi1))/m 0 0 0;
                cos(phi1(j))*cos(theta1(l))/m 0 0 0;
                0 1/Jx 0 0;
                0 0 1/Jy 0;
                0 0 0 1/Jz];
            sum= sum+1;
                  end
             end
        end
    end
end
