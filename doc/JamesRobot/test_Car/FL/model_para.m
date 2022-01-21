clc;clear;close all
%% parameter
beta = 10;
Ar = -beta*eye(7);
%%
% x(1) = cos(phi)
% x(2) = sin(phi)
% x(3) = wr

J1_1 = @(x) 0;
J1_2 = @(x) 0;
J1_3 = @(x) 0;
J1_4 = @(x) (x(1)-1);
J1_5 = @(x) 0;
J1_6 = @(x) 0;
J1_7 = @(x) 0;
J1_8 = @(x) 0;
J1_9 = @(x) x(3);
J1_10 = @(x) 0;
J1_11 = @(x) x(1);
J1_12 = @(x) 0;

J2_1 = @(x) 0;
J2_2 = @(x) 0;
J2_3 = @(x) 0;
J2_4 = @(x) x(2);
J2_5 = @(x) 0;
J2_6 = @(x) 0;
J2_7 = @(x) 0;
J2_8 = @(x) -x(3);
J2_9 = @(x) 0;
J2_10 = @(x) 0;
J2_11 = @(x) x(2);
J2_12 = @(x) 0;



save('model_para.mat');