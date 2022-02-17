%% find parameters multi-UAV
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

%% FUZZY interpolation %%
pv1 = [-20 0 20];  %% phi
pv2 = [-20 0 20];  %% theta 
pv3 = [-20 0 20];  %% psi
FindLocalMatrix();

%% LMI find K %%
rho=18;
%Q = eye(12);
Q = zeros(24,12);
Q(1:12,:) = 0.01*diag([1,0.001,1,0.001,1,0.001,1,0.001,1,0.001,1,0.001]);
Q(13:24,:) = 0.01*diag([1,0.001,1,0.001,1,0.001,1,0.001,1,0.001,1,0.001]);
LMI_find_K_multi(rho,Q);

%% check system stability %%
check_stability();
save multi_UAV.mat;