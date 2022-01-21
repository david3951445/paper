clc;clear;close all
% Conclusion: Let x(1) = phi0-phi1. 
% If use phi0 and phi1 as premise variable, there will appear singular
% value. 
%% 
% x(1) = cos(ePhi1), x(2) = sin(ePhi1), x(3) = cos(PHI), x(4) = sin(PHI), 
% x(5) = cos(Phi0r-Phi1r), x(6) = w1r
%% parameter
d = 0.2;
%% 
J1_1 = @(x) 0;
J1_2 = @(x) 0;
J1_3 = @(x) 0;
J1_4 = @(x) 0;
J1_5 = @(x) x(1)*x(3) - x(5);
J1_6 = @(x) 0;
J1_7 = @(x) 0;
J1_8 = @(x) 0;
J1_9 = @(x) 0;
J1_10 = @(x) 0;
J1_11 = @(x) x(6);
J1_12 = @(x) 0;
J1_13 = @(x) 0;
J1_14 = @(x) x(1)*x(3);
J1_15 = @(x) 0;

J2_1 = @(x) 0;
J2_2 = @(x) 0;
J2_3 = @(x) 0;
J2_4 = @(x) 0;
J2_5 = @(x) x(2)*x(3);
J2_6 = @(x) 0;
J2_7 = @(x) 0;
J2_8 = @(x) 0;
J2_9 = @(x) 0;
J2_10 = @(x) -x(6);
J2_11 = @(x) 0;
J2_12 = @(x) 0;
J2_13 = @(x) 0;
J2_14 = @(x) x(2)*x(3);
J2_15 = @(x) 0;

J3_1 = @(x) 0;
J3_2 = @(x) 0;
J3_3 = @(x) 0;
J3_4 = @(x) 0;
J3_5 = @(x) x(4)/d;
J3_6 = @(x) 0;
J3_7 = @(x) -1;
J3_8 = @(x) 0;
J3_9 = @(x) 0;
J3_10 = @(x) 0;
J3_11 = @(x) 0;
J3_12 = @(x) 0;
J3_13 = @(x) 0;
J3_14 = @(x) x(4)/d;
J3_15 = @(x) 0;

J4_1 = @(x) 0;
J4_2 = @(x) 0;
J4_3 = @(x) 0;
J4_4 = @(x) 0;
J4_5 = @(x) 0;
J4_6 = @(x) 0;
J4_7 = @(x) 0;
J4_8 = @(x) 0;
J4_9 = @(x) 0;
J4_10 = @(x) 0;
J4_11 = @(x) 0;
J4_12 = @(x) 0;
J4_13 = @(x) 0;
J4_14 = @(x) 0;
J4_15 = @(x) 1;

save('model_para_part1_v8.mat');