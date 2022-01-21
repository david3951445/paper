clc;clear;close all
load('model_para_part2_v2.mat');
addpath 'G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot'
%% for x(5)
%% lpv
lpv = {...
    @(x)G1_1(x)   @(x)G1_2(x)   @(x)G1_3(x)   @(x)G1_4(x)   @(x)G1_5(x)   @(x)G1_6(x)   @(x)G1_7(x)   @(x)G1_8(x)   @(x)G1_9(x)   @(x)G1_10(x)  @(x)G1_11(x)  @(x)G1_12(x)  @(x)G1_13(x)  @(x)G1_14(x)  @(x)G1_15(x)  @(x)G1_16(x)  @(x)G1_17(x)  @(x)G1_18(x)  @(x)G1_19(x)  @(x)G1_20(x)  @(x)G1_21(x)  @(x)G1_22(x)  @(x)G1_23(x)  @(x)G1_24(x)  @(x)G1_25(x)  @(x)G1_26(x)  @(x)G1_27(x)  @(x)G1_28(x)  ;
    @(x)G2_1(x)   @(x)G2_2(x)   @(x)G2_3(x)   @(x)G2_4(x)   @(x)G2_5(x)   @(x)G2_6(x)   @(x)G2_7(x)   @(x)G2_8(x)   @(x)G2_9(x)   @(x)G2_10(x)  @(x)G2_11(x)  @(x)G2_12(x)  @(x)G2_13(x)  @(x)G2_14(x)  @(x)G2_15(x)  @(x)G2_16(x)  @(x)G2_17(x)  @(x)G2_18(x)  @(x)G2_19(x)  @(x)G2_20(x)  @(x)G2_21(x)  @(x)G2_22(x)  @(x)G2_23(x)  @(x)G2_24(x)  @(x)G2_25(x)  @(x)G2_26(x)  @(x)G2_27(x)  @(x)G2_28(x)  ;
    @(x)G3_1(x)   @(x)G3_2(x)   @(x)G3_3(x)   @(x)G3_4(x)   @(x)G3_5(x)   @(x)G3_6(x)   @(x)G3_7(x)   @(x)G3_8(x)   @(x)G3_9(x)   @(x)G3_10(x)  @(x)G3_11(x)  @(x)G3_12(x)  @(x)G3_13(x)  @(x)G3_14(x)  @(x)G3_15(x)  @(x)G3_16(x)  @(x)G3_17(x)  @(x)G3_18(x)  @(x)G3_19(x)  @(x)G3_20(x)  @(x)G3_21(x)  @(x)G3_22(x)  @(x)G3_23(x)  @(x)G3_24(x)  @(x)G3_25(x)  @(x)G3_26(x)  @(x)G3_27(x)  @(x)G3_28(x)  
};


%% dep
dep = zeros([size(lpv) 3]);
%% x(1)
dep(:,:,1) = [...
    0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
];
%% x(2)
dep(:,:,2) = [...
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
];
%% x(3)
dep(:,:,3) = [...
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
];
domain = [-1,1;-1,1;-0.06,0.06];
gridsize = [151,151,151];

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


save('lpv_data_part2_v2', 'lpvdata', 'S', 'U', 'domain', 'gridsize');

%%
P = size(domain,1);
A = cell(size(S,1:P));
%%
% pass
for i = 1:size(S,1)
    for j = 1:size(S,2)
        for k = 1:size(S,3)
            A{i,j,k}(:,:) = S(i,j,k,:,:);
        end        
    end
end