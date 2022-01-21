clc;clear;close all
load('model_para_part1_v8.mat');
addpath 'G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot'
% for x(1) ~ x(4)
%% lpv
lpv = {...
    @(x)J1_1(x),@(x)J1_2(x),@(x)J1_3(x),@(x)J1_4(x),@(x)J1_5(x),@(x)J1_6(x),@(x)J1_7(x),@(x)J1_8(x),@(x)J1_9(x),@(x)J1_10(x),@(x)J1_11(x),@(x)J1_12(x),@(x)J1_13(x),@(x)J1_14(x),@(x)J1_15(x);     
    @(x)J2_1(x),@(x)J2_2(x),@(x)J2_3(x),@(x)J2_4(x),@(x)J2_5(x),@(x)J2_6(x),@(x)J2_7(x),@(x)J2_8(x),@(x)J2_9(x),@(x)J2_10(x),@(x)J2_11(x),@(x)J2_12(x),@(x)J2_13(x),@(x)J2_14(x),@(x)J2_15(x);      
    @(x)J3_1(x),@(x)J3_2(x),@(x)J3_3(x),@(x)J3_4(x),@(x)J3_5(x),@(x)J3_6(x),@(x)J3_7(x),@(x)J3_8(x),@(x)J3_9(x),@(x)J3_10(x),@(x)J3_11(x),@(x)J3_12(x),@(x)J3_13(x),@(x)J3_14(x),@(x)J3_15(x);     
    @(x)J4_1(x),@(x)J4_2(x),@(x)J4_3(x),@(x)J4_4(x),@(x)J4_5(x),@(x)J4_6(x),@(x)J4_7(x),@(x)J4_8(x),@(x)J4_9(x),@(x)J4_10(x),@(x)J4_11(x),@(x)J4_12(x),@(x)J4_13(x),@(x)J4_14(x),@(x)J4_15(x);                     
    };


%% dep
dep = zeros([size(lpv) 6]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% x = [phi0 phi1 w0 w1 phiBR vBR th vxr vyr phi1r wr];4 8 8 2 1 4
% x = [1    2    3  4  5     6   7  8   9   10    11];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% x(1)
dep(:,:,1) = [...
    0,0,0,0,1,0,0,0,0,0,0,0,0,1,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
];

%% x(2)
dep(:,:,2) = [...
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,1,0,0,0,0,0,0,0,0,1,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
];
%% x(3)
dep(:,:,3) = [...
    0,0,0,0,1,0,0,0,0,0,0,0,0,1,0;
    0,0,0,0,1,0,0,0,0,0,0,0,0,1,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
];
%% x(4)
dep(:,:,4) = [...
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,1,0,0,0,0,0,0,0,0,1,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
];
%% x(5)
dep(:,:,5) = [...
    0,0,0,0,1,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
];
%% x(6)
dep(:,:,6) = [...
    0,0,0,0,0,0,0,0,0,0,1,0,0,0,0;
    0,0,0,0,0,0,0,0,0,1,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
];
%% domain & grid size
% sampling intervals for each parameter
domain = [-1,1;-1,1;-1,1;-1,1;-1,1;-0.06,0.06];
% grid size: number of grid points for each parameter
gridsize = [151,151,151,151,151,151];

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

save('lpv_data_part1_v8', 'lpvdata', 'S', 'U', 'domain', 'gridsize');

%%
P = size(domain,1);
A = cell(size(S,1:P));
%%
% pass
for i = 1:size(S,1)
    for j = 1:size(S,2)
        for k = 1:size(S,2)
            for l = 1:size(S,2)
                for m = 1:size(S,2)
                    for n = 1:size(S,2)
                        A{i,j,k,l,m,n}(:,:) = S(i,j,k,l,m,n,:,:);
                    end
                end
            end
        end
    end
end




