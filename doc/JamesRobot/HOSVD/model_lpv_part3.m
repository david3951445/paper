clc;clear;close all
load('model_para_part3.mat');
 
addpath 'G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot'
%% lpv
lpv = {...
    @(x)Je1_1(x)   @(x)Je1_2(x)   @(x)Je1_3(x)   @(x)Je1_4(x)   @(x)Je1_5(x)   @(x)Je1_6(x)   @(x)Je1_7(x)   @(x)Je1_8(x)   @(x)Je1_9(x)   @(x)Je1_10(x)  @(x)Je1_11(x)  @(x)Je1_12(x)  ;
    @(x)Je2_1(x)   @(x)Je2_2(x)   @(x)Je2_3(x)   @(x)Je2_4(x)   @(x)Je2_5(x)   @(x)Je2_6(x)   @(x)Je2_7(x)   @(x)Je2_8(x)   @(x)Je2_9(x)   @(x)Je2_10(x)  @(x)Je2_11(x)  @(x)Je2_12(x)  ;
    @(x)Je3_1(x)   @(x)Je3_3(x)   @(x)Je3_3(x)   @(x)Je3_4(x)   @(x)Je3_5(x)   @(x)Je3_6(x)   @(x)Je3_7(x)   @(x)Je3_8(x)   @(x)Je3_9(x)   @(x)Je3_10(x)  @(x)Je3_11(x)  @(x)Je3_12(x)  ;
    @(x)Je4_1(x)   @(x)Je4_4(x)   @(x)Je4_3(x)   @(x)Je4_4(x)   @(x)Je4_5(x)   @(x)Je4_6(x)   @(x)Je4_7(x)   @(x)Je4_8(x)   @(x)Je4_9(x)   @(x)Je4_10(x)  @(x)Je4_11(x)  @(x)Je4_12(x)  ;
    @(x)Je5_1(x)   @(x)Je5_5(x)   @(x)Je5_3(x)   @(x)Je5_4(x)   @(x)Je5_5(x)   @(x)Je5_6(x)   @(x)Je5_7(x)   @(x)Je5_8(x)   @(x)Je5_9(x)   @(x)Je5_10(x)  @(x)Je5_11(x)  @(x)Je5_12(x)  ;
    @(x)Je6_1(x)   @(x)Je6_6(x)   @(x)Je6_3(x)   @(x)Je6_4(x)   @(x)Je6_5(x)   @(x)Je6_6(x)   @(x)Je6_7(x)   @(x)Je6_8(x)   @(x)Je6_9(x)   @(x)Je6_10(x)  @(x)Je6_11(x)  @(x)Je6_12(x)  ;
    @(x)Je7_1(x)   @(x)Je7_7(x)   @(x)Je7_3(x)   @(x)Je7_4(x)   @(x)Je7_5(x)   @(x)Je7_6(x)   @(x)Je7_7(x)   @(x)Je7_8(x)   @(x)Je7_9(x)   @(x)Je7_10(x)  @(x)Je7_11(x)  @(x)Je7_12(x)  ;
    @(x)Je8_1(x)   @(x)Je8_8(x)   @(x)Je8_3(x)   @(x)Je8_4(x)   @(x)Je8_5(x)   @(x)Je8_6(x)   @(x)Je8_7(x)   @(x)Je8_8(x)   @(x)Je8_9(x)   @(x)Je8_10(x)  @(x)Je8_11(x)  @(x)Je8_12(x)  ;
    @(x)Je9_1(x)   @(x)Je9_9(x)   @(x)Je9_3(x)   @(x)Je9_4(x)   @(x)Je9_5(x)   @(x)Je9_6(x)   @(x)Je9_7(x)   @(x)Je9_8(x)   @(x)Je9_9(x)   @(x)Je9_10(x)  @(x)Je9_11(x)  @(x)Je9_12(x)  ;
    @(x)Je10_1(x)  @(x)Je10_2(x)  @(x)Je10_3(x)  @(x)Je10_4(x)  @(x)Je10_5(x)  @(x)Je10_6(x)  @(x)Je10_7(x)  @(x)Je10_8(x)  @(x)Je10_9(x)  @(x)Je10_10(x) @(x)Je10_11(x) @(x)Je10_12(x) ;
    @(x)Je11_1(x)  @(x)Je11_2(x)  @(x)Je11_3(x)  @(x)Je11_4(x)  @(x)Je11_5(x)  @(x)Je11_6(x)  @(x)Je11_7(x)  @(x)Je11_8(x)  @(x)Je11_9(x)  @(x)Je11_10(x) @(x)Je11_11(x) @(x)Je11_12(x) ;
    @(x)Je12_1(x)  @(x)Je12_2(x)  @(x)Je12_3(x)  @(x)Je12_4(x)  @(x)Je12_5(x)  @(x)Je12_6(x)  @(x)Je12_7(x)  @(x)Je12_8(x)  @(x)Je12_9(x)  @(x)Je12_10(x) @(x)Je12_11(x) @(x)Je12_12(x) ;
    @(x)Gbar1_1(x)  @(x)Gbar1_2(x)  @(x)Gbar1_3(x)  @(x)Gbar1_4(x)  @(x)Gbar1_5(x)  @(x)Gbar1_6(x)  @(x)Gbar1_7(x)  @(x)Gbar1_8(x)  @(x)Gbar1_9(x)  @(x)Gbar1_10(x)  @(x)Gbar1_11(x)  @(x)Gbar1_12(x) ;
    @(x)Gbar2_1(x)  @(x)Gbar2_2(x)  @(x)Gbar2_3(x)  @(x)Gbar2_4(x)  @(x)Gbar2_5(x)  @(x)Gbar2_6(x)  @(x)Gbar2_7(x)  @(x)Gbar2_8(x)  @(x)Gbar2_9(x)  @(x)Gbar2_10(x)  @(x)Gbar2_11(x)  @(x)Gbar2_12(x) ;
    @(x)Gbar3_1(x)  @(x)Gbar3_2(x)  @(x)Gbar3_3(x)  @(x)Gbar3_4(x)  @(x)Gbar3_5(x)  @(x)Gbar3_6(x)  @(x)Gbar3_7(x)  @(x)Gbar3_8(x)  @(x)Gbar3_9(x)  @(x)Gbar3_10(x)  @(x)Gbar3_11(x)  @(x)Gbar3_12(x) ;
    @(x)Gbar4_1(x)  @(x)Gbar4_2(x)  @(x)Gbar4_3(x)  @(x)Gbar4_4(x)  @(x)Gbar4_5(x)  @(x)Gbar4_6(x)  @(x)Gbar4_7(x)  @(x)Gbar4_8(x)  @(x)Gbar4_9(x)  @(x)Gbar4_10(x)  @(x)Gbar4_11(x)  @(x)Gbar4_12(x) ;
    @(x)Gbar5_1(x)  @(x)Gbar5_2(x)  @(x)Gbar5_3(x)  @(x)Gbar5_4(x)  @(x)Gbar5_5(x)  @(x)Gbar5_6(x)  @(x)Gbar5_7(x)  @(x)Gbar5_8(x)  @(x)Gbar5_9(x)  @(x)Gbar5_10(x)  @(x)Gbar5_11(x)  @(x)Gbar5_12(x) ;
    @(x)Gbar6_1(x)  @(x)Gbar6_2(x)  @(x)Gbar6_3(x)  @(x)Gbar6_4(x)  @(x)Gbar6_5(x)  @(x)Gbar6_6(x)  @(x)Gbar6_7(x)  @(x)Gbar6_8(x)  @(x)Gbar6_9(x)  @(x)Gbar6_10(x)  @(x)Gbar6_11(x)  @(x)Gbar6_12(x) ;
    @(x)Gbar7_1(x)  @(x)Gbar7_2(x)  @(x)Gbar7_3(x)  @(x)Gbar7_4(x)  @(x)Gbar7_5(x)  @(x)Gbar7_6(x)  @(x)Gbar7_7(x)  @(x)Gbar7_8(x)  @(x)Gbar7_9(x)  @(x)Gbar7_10(x)  @(x)Gbar7_11(x)  @(x)Gbar7_12(x) ;
    @(x)Gbar8_1(x)  @(x)Gbar8_2(x)  @(x)Gbar8_3(x)  @(x)Gbar8_4(x)  @(x)Gbar8_5(x)  @(x)Gbar8_6(x)  @(x)Gbar8_7(x)  @(x)Gbar8_8(x)  @(x)Gbar8_9(x)  @(x)Gbar8_10(x)  @(x)Gbar8_11(x)  @(x)Gbar8_12(x) ;
    @(x)Gbar9_1(x)  @(x)Gbar9_2(x)  @(x)Gbar9_3(x)  @(x)Gbar9_4(x)  @(x)Gbar9_5(x)  @(x)Gbar9_6(x)  @(x)Gbar9_7(x)  @(x)Gbar9_8(x)  @(x)Gbar9_9(x)  @(x)Gbar9_10(x)  @(x)Gbar9_11(x)  @(x)Gbar9_12(x) ;
    @(x)Gbar10_1(x) @(x)Gbar10_2(x) @(x)Gbar10_3(x) @(x)Gbar10_4(x) @(x)Gbar10_5(x) @(x)Gbar10_6(x) @(x)Gbar10_7(x) @(x)Gbar10_8(x) @(x)Gbar10_9(x) @(x)Gbar10_10(x) @(x)Gbar10_11(x) @(x)Gbar10_12(x);
    @(x)Gbar11_1(x) @(x)Gbar11_2(x) @(x)Gbar11_3(x) @(x)Gbar11_4(x) @(x)Gbar11_5(x) @(x)Gbar11_6(x) @(x)Gbar11_7(x) @(x)Gbar11_8(x) @(x)Gbar11_9(x) @(x)Gbar11_10(x) @(x)Gbar11_11(x) @(x)Gbar11_12(x);
    @(x)Gbar12_1(x) @(x)Gbar12_2(x) @(x)Gbar12_3(x) @(x)Gbar12_4(x) @(x)Gbar12_5(x) @(x)Gbar12_6(x) @(x)Gbar12_7(x) @(x)Gbar12_8(x) @(x)Gbar12_9(x) @(x)Gbar12_10(x) @(x)Gbar12_11(x) @(x)Gbar12_12(x)
};
%% dep
dep = zeros([size(lpv) 6]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% x = [phi0 phi1 w0 w1 phiBR vBR th vxr vyr phi1r wr];4 8 8 2 1 4
% x = [1    2    3  4  5     6   7  8   9   10    11];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% x(6)
dep(1:12,1:12,1) = [...
    0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0;
    1,0,0,0,0,0,0,0,0,0,0,0;
    1,0,0,0,0,0,0,0,0,0,0,0;
    1,0,0,0,0,0,0,0,0,0,0,0;
    1,0,0,0,0,0,0,0,0,0,0,0;
    1,0,0,0,0,0,0,0,0,0,0,0;
    1,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0
];
%% x(7)
dep(1:12,1:12,2) = [...
    0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0;
    1,0,0,0,0,0,0,0,0,0,0,0;
    1,0,0,0,0,0,0,0,0,0,0,0;
    1,0,0,0,0,0,0,0,0,0,0,0;
    1,0,0,0,0,0,0,0,0,0,0,0;
    1,0,0,0,0,0,0,0,0,0,0,0;
    1,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0
];

dep(21:24,1:12,2) = [...
    0,0,0,0,1,1,0,0,0,0,0,0;
    0,0,0,0,1,1,0,0,0,0,0,0;
    0,0,0,0,1,1,0,0,0,0,0,0;
    0,0,0,0,1,1,0,0,0,0,0,0
];

%% x(8)
dep(21:24,1:12,3) = [...
    0,0,0,0,1,1,0,0,0,0,0,0;
    0,0,0,0,1,1,0,0,0,0,0,0;
    0,0,0,0,1,1,0,0,0,0,0,0;
    0,0,0,0,1,1,0,0,0,0,0,0
];
%% x(9)
dep(21:24,1:12,4) = [...
    0,0,0,0,1,1,0,0,0,0,0,0;
    0,0,0,0,1,1,0,0,0,0,0,0;
    0,0,0,0,1,1,0,0,0,0,0,0;
    0,0,0,0,1,1,0,0,0,0,0,0
];
%% x(10)
dep(13:20,1:12,5) = [...
    0,0,0,0,1,1,0,0,1,1,0,0;
    0,0,0,0,1,1,0,0,1,1,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0
];
%% x(11)
dep(13:20,1:12,6) = [...
    0,0,0,0,1,1,0,0,0,0,0,0;
    0,0,0,0,1,1,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0;
    0,0,0,0,0,0,0,0,0,0,0,0
];
%% domain & grid size
% sampling intervals for each parameter
domain = [0 0.15; 1 8001; 0 0.2; 0 0.1; -pi/2 pi/2; -0.1 0.1];
% grid size: number of grid points for each parameter
gridsize = [31 101 31 31 31 31];

%% TP transformation, same as:
%   [S U] = tptrans(lpv, dep, domain, gridsize, 'close');

% sampling
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 這邊不好撒點--> 因為index要是正整數
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% check model approximation error
[maxerr meanerr] = tperror(lpv, S, U, domain, 100);

disp('max and mean error:'); disp(maxerr); disp(meanerr);

save('lpv_data_part3','lpvdata', 'S', 'U', 'domain', 'gridsize');
%%
P = size(domain,1);
A = cell(size(S,1:P));
%% 
% pass
for i = 1:size(S,1)
    for j = 1:size(S,2)
        for k = 1:size(S,3)
            for l = 1:size(S,4)
                for m = 1:size(S,5)
                    for n = 1:size(S,6)
                        A{i,j,k,l,m,n}(:,:) = S(i,j,k,l,m,n,:,:);
                    end
                end
            end
        end
    end
end
