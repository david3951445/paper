clc;clear;close all
% Conclusion: Let x(1) = phi0-phi1. 
% If use phi0 and phi1 as premise variable, there will appear singular
% value. 
%% x = phi0
%% parameter
beta = 10;
Ar = -beta*eye(7);

%% only y
% Jt2_1 = @(x) sin(x(1));

%% only x
% Jt2_1 = @(x) cos(x(1));

%% x and y 
% % x(1) = phi0
Jt1_1 = @(x) cos(x(1));
Jt2_1 = @(x) sin(x(1));

%%
% x(1) = phi0, x(2) = v, x(3) = w --> 強制讓phi和x,y有關係,用這個找K帶回原本的
% Jt1_1 = @(x) -x(2)*sin(x(1))*x(3);
% Jt2_1 = @(x) x(2)*cos(x(1))*x(3);
% Jt1_2 = @(x) cos(x(1));
% Jt2_2 = @(x) sin(x(1));


save('model_para.mat');