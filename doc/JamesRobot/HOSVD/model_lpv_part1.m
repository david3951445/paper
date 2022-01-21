clc;clear;close all
load('model_para_part1_v2.mat');
addpath 'G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot'
%% for x(1) ~ x(4)
%% lpv
lpv = {...
    @(x)Jt1_1(x)     @(x)Jt2_1(x)      @(x)Jt3_1(x)     @(x)Jt4_1(x)  @(x)J_bar1_1(x) @(x)J_bar1_2(x) @(x)J_bar1_3(x) @(x)J_bar1_4(x) @(x)J_bar1_5(x) @(x)J_bar1_6(x) @(x)J_bar1_7(x) @(x)J_bar1_8(x) @(x)J_bar1_9(x) @(x)J_bar1_10(x) @(x)J_bar1_11(x) @(x)J_bar1_12(x)  @(x)J_bar1_13(x)  @(x)J_bar1_14(x)  @(x)J_bar1_15(x)  @(x)-S7Jt1_1(x) @(x)-S7Jt2_1(x) @(x)-S7Jt3_1(x); 
    @(x)Jt1_2(x)     @(x)Jt2_2(x)      @(x)Jt3_2(x)     @(x)Jt4_2(x)  @(x)J_bar2_1(x) @(x)J_bar2_2(x) @(x)J_bar2_3(x) @(x)J_bar2_4(x) @(x)J_bar2_5(x) @(x)J_bar2_6(x) @(x)J_bar2_7(x) @(x)J_bar2_8(x) @(x)J_bar2_9(x) @(x)J_bar2_10(x) @(x)J_bar2_11(x) @(x)J_bar2_12(x)  @(x)J_bar2_13(x)  @(x)J_bar2_14(x)  @(x)J_bar2_15(x)  @(x)-S7Jt1_2(x) @(x)-S7Jt2_2(x) @(x)-S7Jt3_2(x)
    };


%% dep
dep = zeros([size(lpv) 4]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% x = [phi0 phi1 w0 w1 phiBR vBR th vxr vyr phi1r wr];4 8 8 2 1 4
% x = [1    2    3  4  5     6   7  8   9   10    11];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% x(1)
dep(1:2,1:4,1) = [...
    1,1,1,0;
    0,0,0,0
];

dep(1:2,5:19,1) = [...
    0,0,0,0,0,1,1,1,0,0,1,1,1,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
];

dep(1:2,20:22,1) = [...
    1,1,1;
    0,0,0
];

%% x(2)
dep(1:2,1:4,2) = [...
    1,1,1,0;
    0,0,0,0
];

dep(1:2,5:19,2) = [...
    0,0,0,0,0,1,1,1,0,0,1,1,1,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
];

dep(1:2,20:22,2) = [...
    1,1,1;
    0,0,0
];


%% x(3)
dep(1:2,5:19,3) = [...
    0,0,0,0,0,1,1,1,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
];

%% x(4)
dep(1:2,5:19,4) = [...
    0,0,0,0,0,1,1,1,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
];
%% domain & grid size
% sampling intervals for each parameter
domain = [-1 1.7;-0.6 1.6;-0.21 0.18;-0.01 0.01];
% grid size: number of grid points for each parameter
gridsize = [151 81 81 21];

%% TP transformation, same as:
%   [S U] = tptrans(lpv, dep, domain, gridsize, 'close');

% sampling
lpvdata = sampling_lpv(lpv, dep, domain, gridsize);
%%
% hosvd
[S U sv tol] = hosvd_lpv(lpvdata, dep, gridsize);

% generating tight polytopic representation
hull = 'close';
U = genhull(U, hull);
S = coretensor(U, lpvdata, dep);

% plot the results
plothull(U, domain);
%%
% check model approximation error
[maxerr meanerr] = tperror(lpv, S, U, domain, 1000);
disp('max and mean error:'); disp(maxerr); disp(meanerr);

save('lpv_data_part1', 'lpvdata', 'S', 'U', 'domain', 'gridsize');

%%
P = size(domain,1);
A = cell(size(S,1:P));
%%
% pass
for i = 1:size(S,1)
    for j = 1:size(S,2)
        for k = 1:size(S,3)
            for l = 1:size(S,4)
                A{i,j,k,l}(:,:) = S(i,j,k,l,:,:);
            end
        end
    end
end




