clc;clear;close all
load('model_para_part1_v5.mat');
addpath 'G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot'
% for x(1) ~ x(4)
%% lpv
lpv = {...
    @(x)Jt1_1(x)     @(x)Jt2_1(x)      @(x)Jt3_1(x)     @(x)Jt4_1(x)    @(x)g1_1(x)    @(x)g1_2(x); 
    @(x)Jt1_2(x)     @(x)Jt2_2(x)      @(x)Jt3_2(x)     @(x)Jt4_2(x)    @(x)g2_1(x)    @(x)g2_2(x)
    };


%% dep
dep = zeros([size(lpv) 2]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% x = [phi0 phi1 w0 w1 phiBR vBR th vxr vyr phi1r wr];4 8 8 2 1 4
% x = [1    2    3  4  5     6   7  8   9   10    11];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% x(1)
dep(:,:,1) = [...
    0,0,1,0,1,0;
    0,0,0,0,0,0
];

%% x(2)
dep(:,:,2) = [...
    1,1,1,0,1,0;
    0,0,0,0,0,0
];

%% domain & grid size
% sampling intervals for each parameter
domain = [-1 1;-1 2];
% grid size: number of grid points for each parameter
gridsize = [151 151];

%% TP transformation, same as:
%   [S U] = tptrans(lpv, dep, domain, gridsize, 'close');

% sampling
lpvdata = sampling_lpv(lpv, dep, domain, gridsize);
%%
% hosvd
[S U sv tol] = hosvd_lpv(lpvdata, dep, gridsize,1e-3);

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

save('lpv_data_part1_v5', 'lpvdata', 'S', 'U', 'domain', 'gridsize');

%%
P = size(domain,1);
A = cell(size(S,1:P));
%%
% pass
for i = 1:size(S,1)
    for j = 1:size(S,2)
        A{i,j}(:,:) = S(i,j,:,:);
    end
end




