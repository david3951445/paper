clc;clear;close all
%% x = [phi0 phi1 w0 w1 phiBR vBR th vxr vyr phi1r wr];4 8 8 2 1 4
%  x = [1    2    3  4  5     6   7  8   9   10    11];
% lpv = {...
%     @(x)Ar(x)       @(x)zeros(12,4) @(x)zeros(12,2) @(x)zeros(12)   @(x)zeros(12,3) @(x)zeros(12)  ;
%     @(x)S3*Ar(x)    @(x)zeros(4)    @(x)Jt(x)       @(x)zeros(4,12) @(x)zeros(4,3)  @(x)zeros(4,12);
%     @(x)J_bar(x)    @(x)zeros(2,4)  @(x)zeros(2,2)  @(x)zeros(2,12) @(x)zeros(2,3)  @(x)zeros(2,12);
%     @(x)zeros(12)   @(x)zeros(12,4) @(x)zeros(12,2) @(x)zeros(12)   @(x)zeros(12,3) @(x)S4*Je(x)   ;
%     @(x)zeros(3,12) @(x)zeros(3,4)  @(x)-S5*Jt(x)   @(x)zeros(3,12) @(x)zeros(3)    @(x)G(x)*S1    ;
%     @(x)G_bar(x)    @(x)zeros(12,4) @(x)zeros(12,2) @(x)zeros(12,12)@(x)zeros(12,3) @(x)zeros(12,12);
% };
%% parameter
S1 = [eye(8) zeros(8,4)];
S2 = [zeros(1,2) eye(1)];
S3 = [eye(4) zeros(4,11)];
S5 = [0,1;1,0];
S4 = [zeros(2,8) S5 zeros(2,5)];
S11 = [eye(4) zeros(4,1)];

d = 0.2;
%% lpv{2,3} 
Jt1_1 = @(x) cos(x(2))*cos(x(1)-x(2));
Jt1_2 = @(x) 0;
Jt2_1 = @(x) sin(x(2))*cos(x(1)-x(2));
Jt2_2 = @(x) 0;
Jt3_1 = @(x) 1/d*sin(x(1)-x(2));
Jt3_2 = @(x) 0;
Jt4_1 = @(x) 0;
Jt4_2 = @(x) 1;

save('model_para_part1_v4.mat');