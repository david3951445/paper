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
S3 = [eye(4) zeros(4,8)];
S4 = [S1' [zeros(4,2) eye(4) [zeros(2) eye(2)]' eye(4)]']';
S5 = [[eye(2) zeros(2)]' [zeros(1,3) eye(1)]']';
S7 = [S5 zeros(3,2)];
S6 = [zeros(6,12) S7' zeros(6,12)]';
S9 = [zeros(4,2) -eye(4) [zeros(2) -eye(2)]']';
S8 = blkdiag(eye(8),S9)';

d = 20;
beta=2000;
Ar=-beta*eye(12);

%% lpv{5,6}
GS1_1 = @(x) cos(x(1));
GS1_2 = @(x) sin(x(1));
GS1_3 = @(x) 0;
GS1_4 = @(x) 0;
GS1_5 = @(x) 0;
GS1_6 = @(x) 0;
GS1_7 = @(x) 0;
GS1_8 = @(x) 0;
GS1_9 = @(x) 0;
GS1_10 = @(x) 0;
GS1_11 = @(x) 0;
GS1_12 = @(x) 0;

GS2_1 = @(x) sin(x(1));
GS2_2 = @(x) -cos(x(1));
GS2_3 = @(x) 0;
GS2_4 = @(x) 0;
GS2_5 = @(x) 0;
GS2_6 = @(x) 0;
GS2_7 = @(x) 0;
GS2_8 = @(x) 0;
GS2_9 = @(x) 0;
GS2_10 = @(x) 0;
GS2_11 = @(x) 0;
GS2_12 = @(x) 0;

GS3_1 = @(x) 0;
GS3_2 = @(x) 0;
GS3_3 = @(x) 0;
GS3_4 = @(x) 0;
GS3_5 = @(x) 0;
GS3_6 = @(x) 0;
GS3_7 = @(x) 0;
GS3_8 = @(x) 1;
GS3_9 = @(x) 0;
GS3_10 = @(x) 0;
GS3_11 = @(x) 0;
GS3_12 = @(x) 0;


save('model_para_part2.mat');