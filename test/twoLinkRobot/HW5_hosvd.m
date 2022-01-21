clc; clear; close all
addpath(genpath('..\..\src'))
rb = Robot();

% tp model
num_p = 4; % length of parameter vector of lpv system
lpv = rb.Al; 
dep = zeros([size(lpv) 4]);
dep(2,1,:) = [1 0 1 0];
dep(2,2,:) = [1 1 1 0];
dep(2,3,:) = [1 0 1 0];
dep(2,4,:) = [1 0 1 1];
dep(4,1,:) = [1 0 1 0];
dep(4,2,:) = [1 1 1 0];
dep(4,3,:) = [1 0 1 0];
dep(4,4,:) = [1 0 1 1];

domain = [-1 1; -1 1; -1 1; -1 1];
gridsize = [10 10 10 10];
lpvdata = sampling_lpv(lpv, dep, domain, gridsize);
% hosvd
[S U sv tol] = hosvd_lpv(lpvdata, dep, gridsize, 0.001);
% generating tight polytopic representation
U = genhull(U, 'close');

% plot the results
plothull(U, domain);

% check model approximation error
[maxerr meanerr] = tperror(lpv, S, U, domain, 100);
disp('max and mean error:'); disp(maxerr); disp(meanerr);

C = [1 0 0 0; 0 0 1 0];

A = cell(size(S,1:size(domain,1)));
for i1 = 1:size(S,1)
    for i2 = 1:size(S,2)
        for i3 = 1:size(S,3)
            for i4 = 1 : size(S, 4)
                A{i1,i2,i3,i4}(:,:) = S(i1,i2,i3,i4,:,:);
            end
        end        
    end
end