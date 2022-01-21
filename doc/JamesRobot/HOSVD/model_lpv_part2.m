clc;clear;close all
load('model_para_part2.mat');
addpath 'G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot'
%% for x(5)
%% lpv
lpv = {...
    @(x)GS1_1(x)   @(x)GS1_2(x)   @(x)GS1_3(x)   @(x)GS1_4(x)   @(x)GS1_5(x)   @(x)GS1_6(x)   @(x)GS1_7(x)   @(x)GS1_8(x)   @(x)GS1_9(x)   @(x)GS1_10(x)  @(x)GS1_11(x)  @(x)GS1_12(x)  ;
    @(x)GS2_1(x)   @(x)GS2_2(x)   @(x)GS2_3(x)   @(x)GS2_4(x)   @(x)GS2_5(x)   @(x)GS2_6(x)   @(x)GS2_7(x)   @(x)GS2_8(x)   @(x)GS2_9(x)   @(x)GS2_10(x)  @(x)GS2_11(x)  @(x)GS2_12(x)  ;
    @(x)GS3_1(x)   @(x)GS3_2(x)   @(x)GS3_3(x)   @(x)GS3_4(x)   @(x)GS3_5(x)   @(x)GS3_6(x)   @(x)GS3_7(x)   @(x)GS3_8(x)   @(x)GS3_9(x)   @(x)GS3_10(x)  @(x)GS3_11(x)  @(x)GS3_12(x)  
};


%% dep
dep = zeros([size(lpv) 1]);
%% x(5)
dep(1:3,1:12,1) = [...
    1,1,0,0,0,0,0,0,0,0,0,0;
    1,1,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0
];
domain = [-pi/3 pi/2];
gridsize = 31;

lpvdata = sampling_lpv(lpv, dep, domain, gridsize);
%%
% hosvd
[S U sv tol] = hosvd_lpv(lpvdata, dep, gridsize, 0.001);
% generating tight polytopic representation
hull = 'close';
U = genhull(U, hull);
S = coretensor(U, lpvdata, dep);

% plot the results
plothull(U, domain);

% check model approximation error
[maxerr meanerr] = tperror(lpv, S, U, domain, 100);
disp('max and mean error:'); disp(maxerr); disp(meanerr);


save('lpv_data_part2', 'lpvdata', 'S', 'U', 'domain', 'gridsize');

%%
P = size(domain,1);
A = cell(size(S,1:P),1);
%%
% pass
for i = 1:size(S,1)
    A{i}(:,:) = S(i,:,:);
end