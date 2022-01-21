                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               clc;clear;
close all
S = load('G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\HOSVD\leg_trajectory_interpolation_sample.mat');
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
S5 = ones(2)-eye(2);
S4 = [zeros(2,8) S5 zeros(2,5)];
S6 = [S1' [zeros(4,2) eye(4) [zeros(2) eye(2)]' eye(4)]']';
S7 = [eye(3) zeros(3,1)];
S9 = [S7 zeros(3,2)];
S8 = [zeros(6,12) S9' zeros(6,12)]';
S13 = [zeros(4,2) -eye(4) [zeros(2) -eye(2)]']';
S10 = blkdiag(eye(8),S13)';
S11 = [eye(4) zeros(4,1)];
S12 = [eye(3) zeros(3,2)];

d = 0.2;
beta = 10;
Ar = -beta*eye(12);
%% lpv{2,3} 
Jt1_1 = @(x) cos(x(2))*cos(x(1)-x(2));
Jt1_2 = @(x) 0;
Jt2_1 = @(x) sin(x(2))*cos(x(1)-x(2));
Jt2_2 = @(x) 0;
Jt3_1 = @(x) 1/d*sin(x(1)-x(2));
Jt3_2 = @(x) 0;
Jt4_1 = @(x) 0;
Jt4_2 = @(x) 1;
%% lpv{3,1}
J_bar1_1 = @(x) 0;
J_bar1_2 = @(x) 0;
J_bar1_3 = @(x) 0;
J_bar1_4 = @(x) 0;
J_bar1_5 = @(x) 0;
J_bar1_6 = @(x) -(-(d^2*sin(2*x(1) - 2*x(2))*(d^2 - 1)*(cos(x(1) - 2*x(2)) ...
    + cos(x(1)))*(x(3) - x(4)))/(d^2*cos(2*x(1) - 2*x(2)) - cos(2*x(1) - 2*x(2))...
    + d^2 + 1)^2);
J_bar1_7 = @(x) -((d^2*sin(2*x(1) - 2*x(2))*(d^2 - 1)*(x(3) - x(4))*(sin(x(1) - 2*x(2)) ...
    - sin(x(1))))/(d^2*cos(2*x(1) - 2*x(2)) - cos(2*x(1) - 2*x(2)) + d^2 + 1)^2);
J_bar1_8 = @(x) -(-(2*d*(d^2 - 1)*(cos(x(1) - x(2)) - ...
    cos(3*x(1) - 3*x(2)))*(x(3) - x(4)))/(cos(4*x(1) - 4*x(2)) - 4*cos(2*x(1) - 2*x(2)) ...
    + 4*d^4*cos(2*x(1) - 2*x(2)) - 2*d^2*cos(4*x(1) - 4*x(2)) + ...
    d^4*cos(4*x(1) - 4*x(2)) + 2*d^2 + 3*d^4 + 3));
J_bar1_9 = @(x) 0;
J_bar1_10 = @(x) 0;
J_bar1_11 = @(x) (d^2*(cos(x(1) - 2*x(2)) + cos(x(1))))/(d^2*cos(2*x(1) - 2*x(2)) - cos(2*x(1) - 2*x(2)) + d^2 + 1);
J_bar1_12 = @(x) -(d^2*(sin(x(1) - 2*x(2)) - sin(x(1))))/(d^2*cos(2*x(1) - 2*x(2)) - cos(2*x(1) - 2*x(2)) + d^2 + 1);
J_bar1_13 = @(x) (2*d*sin(x(1) - x(2)))/(d^2*cos(2*x(1) - 2*x(2)) - cos(2*x(1) - 2*x(2)) + d^2 + 1);
J_bar1_14 = @(x) 0;
J_bar1_15 = @(x) 0;
J_bar2_1 = @(x) 0;
J_bar2_2 = @(x) 0;
J_bar2_3 = @(x) 0;
J_bar2_4 = @(x) 0;
J_bar2_5 = @(x) 0;
J_bar2_6 = @(x) 0;
J_bar2_7 = @(x) 0;
J_bar2_8 = @(x) 0;
J_bar2_9 = @(x) 0;
J_bar2_10 = @(x) 0;
J_bar2_11 = @(x) 0;
J_bar2_12 = @(x) 0;
J_bar2_13 = @(x) 0;
J_bar2_14 = @(x) 1;
J_bar2_15 = @(x) 0;
%% lpv{5,3}
S7Jt1_1 = @(x) cos(x(1) - x(2))*cos(x(2));
S7Jt1_2 = @(x) 0;
S7Jt2_1 = @(x) cos(x(1) - x(2))*sin(x(2));
S7Jt2_2 = @(x) 0;
S7Jt3_1 = @(x) sin(x(1) - x(2))/d;
S7Jt3_2 = @(x) 0;

save('model_para_part1_v2.mat');