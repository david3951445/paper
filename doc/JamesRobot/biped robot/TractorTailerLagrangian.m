clc;clear;close all 

load("G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot\com0_trajectory_reference_model_team1_leader.mat");
%% set parameter
phi0 = r(19,:);
phi1 = r(7,:);
w0 = r(20,:);
w1 = r(8,:);

v0 = r(23,:);
a0 = r(24,:);
afa0 = r(21,:);

u = [v0;w0];
du = [a0;afa0];

m0 = 1.47;
m1 = 1;
I0 = 0.005116;
I1 = 0.043416;
d = 0.2;
l = 0.03;
b = 0.06;
rad = 0.025;

B = 1/rad*[1,1;b,-b];

M = zeros(2,2,length(t));
C = zeros(2,2,length(t));
tau = zeros(2,length(t));
%% Lagrangian
for i = 1:length(t)
    tic;
    dPhi = phi0(i)-phi1(i);
    dW = w0(i) - w1(i);
    M(1,1,i) = m0*(cos(dPhi))^2 + I1*(sin(dPhi))^2/d^2;
    M(2,2,i) = I0;
    C(1,1,i) = -m0*cos(dPhi)*sin(dPhi)*dW + I1*sin(dPhi)*cos(dPhi)*dW/d^2;
    C(1,2,i) = -m1*l*w0(i);
    C(2,1,i) = m1*l*w0(i);
    
    tau(:,i) = B\(M(:,:,i)*du(:,i) + C(:,:,i)*u(:,i));
    disp(i);
    toc;
end
%% figure verification
% Remark: Since the trajectory of the tractor-trailer is linear. Therefore,
% the tau1 is same as tau2
figure;
plot(t,tau(1,:));
figure;
plot(t,tau(2,:));
%%
figure;
plot(t,v0);
figure;
plot(t,w0);