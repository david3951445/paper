clc;clear;close all
load('model_para.mat');
addpath 'G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot'

%% lpv
lpv = {...
    @(x)J1_1(x),  @(x)J1_2(x),  @(x)J1_3(x),  @(x)J1_4(x),  @(x)J1_5(x),  @(x)J1_6(x),  @(x)J1_7(x),  @(x)J1_8(x),  @(x)J1_9(x),  @(x)J1_10(x),  @(x)J1_11(x),  @(x)J1_12(x); 
    @(x)J2_1(x),  @(x)J2_2(x),  @(x)J2_3(x),  @(x)J2_4(x),  @(x)J2_5(x),  @(x)J2_6(x),  @(x)J2_7(x),  @(x)J2_8(x),  @(x)J2_9(x),  @(x)J2_10(x),  @(x)J2_11(x),  @(x)J2_12(x)  
    };


%% dep
dep = zeros([size(lpv) 3]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% x = [phi0 phi1 w0 w1 phiBR vBR th vxr vyr phi1r wr];4 8 8 2 1 4
% x = [1    2    3  4  5     6   7  8   9   10    11];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% x(1)
dep(:,:,1) = [...
    0,0,0,1,0,0,0,0,0,0,1,0;
    0,0,0,0,0,0,0,0,0,0,0,0
];

%% x(2)
dep(:,:,2) = [...
    0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,1,0,0,0,0,0,0,1,0
];

%% x(3)
dep(:,:,3) = [...
    0,0,0,0,0,0,0,0,1,0,0,0;
    0,0,0,0,0,0,0,1,0,0,0,0
];
%% domain & grid size
% sampling intervals for each parameter
domain = [-1,1;-1,1;-0.2,0.2];
% grid size: number of grid points for each parameter
gridsize = [201,201,151];

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

save('lpv_data', 'lpvdata', 'S', 'U', 'domain', 'gridsize');

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




