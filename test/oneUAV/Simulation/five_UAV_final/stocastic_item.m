% Runge-Kutta-Fehlberg method on nonlinear stochastic ONE UAV tracking model
clc; clear all; close all;
global Kx Ky Kz Kphi Ktheta Kpsi Jphi Jtheta Jpsi m g Ar mar_stable num
global pv1 pv2 pv3 Local
load multi_UAV.mat;

%% 家览丁把计 ㎝ 繦诀把计 %%
dt = 1e-3;T = 50; N = T/dt; 
R = 2; Dt = R*dt; L = N/R; % L steps size Dt = R*dt

h = 2*dt;      %% Runge-Kutta
errth = 5e-4;  %% Runge-Kutta
beta = 1;      %% Runge-Kutta

%% 倒 initial value %%
index = [1 1 1 1];
Xt_L = [0 0 0 0 0 0 0 0 0 0 0 0]';
Xt_F1 = [0.2 0 0.2 0 0 0 0 0 0 0 0 0]';
Xt_F2 = [-0.2 0 0.2 0 0 0 0 0 0 0 0 0]';
Xt_F3 = [-0.2 0 -0.2 0 0 0 0 0 0 0 0 0]';
Xt_F4 = [0.2 0 -0.2 0 0 0 0 0 0 0 0 0]';
d = zeros(4,L);
n1=0;n2=1;n3=2;n4=3;n5=4;

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
W5 = cumsum(dW5);               %% for weiner 5
Count = 500;                    %% for poisson
lamda = 0.1;                    %% for poisson
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
dp1 = 1; ep1 = 1; level1 = 0;
dp2 = 1; ep2 = 1; level2 = 0;
dp3 = 1; ep3 = 1; level3 = 0;
dp4 = 1; ep4 = 1; level4 = 0;
dp5 = 1; ep5 = 1; level5 = 0;

save stocastic_item.mat;