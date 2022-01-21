%% lpv
lpv = {...
    @(x)Je1_1(x);
    @(x)Je2_1(x);
    @(x)Je3_1(x);
    @(x)Je4_1(x)
};
%% dep
dep = zeros([size(lpv) 2]);
%% x(1)
dep(:,:,1) = [...
    1;
    1;
    1;
    1
];
%% x(2)
dep(:,:,2) = [...
    1;
    1;
    1;
    1
];
%% domain & grid size
% sampling intervals for each parameter
domain = [0,0.2; 1,(S.len+1)];
% grid size: number of grid points for each parameter
gridsize = [101 201];

%% TP transformation, same as:
%   [S U] = tptrans(lpv, dep, domain, gridsize, 'close');
% sampling
lpvdata = sampling_lpv(lpv, dep, domain, gridsize);
%%
% hosvd
[S U sv tol] = hosvd_lpv(lpvdata, dep, gridsize, 0.001);

% generating tight polytopic representation
hull = 'snnn';
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

save('lpv_data_part3_v4','lpvdata', 'S', 'U', 'domain', 'gridsize');
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
