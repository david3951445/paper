clc;clear;close all
% Conclusion: Let x(1) = phi0-phi1. 
% If use phi0 and phi1 as premise variable, there will appear singular
% value. 
%% x = phi0-phi1
%% parameter
d = 0.2;
%% lpv{2,3}
Jt1_1 = @(x) 1/d*sin(x(1));
Jt2_1 = @(x) 0;
Jt1_2 = @(x) 0;
Jt2_2 = @(x) 1;
save('model_para_part1_v7.mat');