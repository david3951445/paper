clc;clear;close all
% Conclusion: Let x(1) = phi0-phi1. 
% If use phi0 and phi1 as premise variable, there will appear singular
% value. 
%% x = [phi0-phi1 phi1 w0 w1 phiBR vBR th vxr vyr phi1r wr];4 8 8 2 1 4
%  x = [1         2    3  4  5     6   7  8   9   10    11];
% lpv = {...
%     @(x)Ar(x)       @(x)zeros(12,4) @(x)zeros(12,2) @(x)zeros(12)   @(x)zeros(12,3) @(x)zeros(12)  ;
%     @(x)S3*Ar(x)    @(x)zeros(4)    @(x)Jt(x)       @(x)zeros(4,12) @(x)zeros(4,3)  @(x)zeros(4,12);
%     @(x)J_bar(x)    @(x)zeros(2,4)  @(x)zeros(2,2)  @(x)zeros(2,12) @(x)zeros(2,3)  @(x)zeros(2,12);
%     @(x)zeros(12)   @(x)zeros(12,4) @(x)zeros(12,2) @(x)zeros(12)   @(x)zeros(12,3) @(x)S4*Je(x)   ;
%     @(x)zeros(3,12) @(x)zeros(3,4)  @(x)-S5*Jt(x)   @(x)zeros(3,12) @(x)zeros(3)    @(x)G(x)*S1    ;
%     @(x)G_bar(x)    @(x)zeros(12,4) @(x)zeros(12,2) @(x)zeros(12,12)@(x)zeros(12,3) @(x)zeros(12,12);
% };
%% parameter
d = 0.2;
%% lpv{2,3} 
Jt1_1 = @(x) cos(x(2));
Jt1_2 = @(x) 0;
Jt2_1 = @(x) sin(x(2));
Jt2_2 = @(x) 0;
Jt3_1 = @(x) 1/d*tan(x(1));
Jt3_2 = @(x) 0;
Jt4_1 = @(x) 0;
Jt4_2 = @(x) 1;
%% g0 
g1_1 = @(x) cos(x(1));
g1_2 = @(x) 0;
g2_1 = @(x) 0;
g2_2 = @(x) 1;
save('model_para_part1_v5.mat');