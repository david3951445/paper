clc;clear;
%% TORA (or RTAC) LPV model

% reference:
% 	R.T. Bupp, D.S. Bernstein, V.T. Coppola
%	A benchmark problem for nonlinear control design
%	International Journal of Robust and Nonlinear Control, 8:307-310 1998

% state vector:
%	x1: position of the cart
%	x2: velocity of the cart
%	x3: angular position of the proof body
%	x4: angular velocity of the proof body

% parameters of the LPV model:
%	p1: x3 (angular position)
%	p2: x4 (angular velocity)

% constant: coupling coefficient between translational and rotational motion
epsilon = 0.2;
% functions in the system matrix
F = @(p) epsilon * p(2) * sin(p(1));
G = @(p) epsilon * cos(p(1));
H = @(p) 1 - epsilon^2 * cos(p(1))^2;
% system matrix: lpv = [A(p) B(p)]
lpv = {...
    @(p)0         @(p)1    @(p)0    @(p)0               @(p)0; 
    @(p)-1/H(p)   @(p)0    @(p)0    @(p)F(p)/H(p)       @(p)-G(p)/H(p);
    @(p)0         @(p)0    @(p)0    @(p)1               @(p)0;
    @(p)G(p)/H(p) @(p)0    @(p)0    @(p)-F(p)*G(p)/H(p) @(p)1/H(p);
};
% number of states (size of the A matrix)
n = 4;
% parameter dependencies:
% dep(i,j,k) is 1 if Sp{i,j} depends on p(k)
dep = zeros([size(lpv) 2]);
dep(2,1,:) = [1 1];
dep(2,4,:) = [1 1];
dep(2,5,:) = [1 1];
dep(4,1,:) = [1 1];
dep(4,4,:) = [1 1];
dep(4,5,:) = [1 1];

% % LPV model
% tora_lpv

% sampling intervals for each parameter
domain = [-45/180*pi 45/180*pi; -0.5 0.5];
% grid size: number of grid points for each parameter
gridsize = [23 23];

%% TP transformation, same as:
%   [S U] = tptrans(lpv, dep, domain, gridsize, 'close');

% sampling
lpvdata = sampling_lpv(lpv, dep, domain, gridsize);

% hosvd
[S U sv tol] = hosvd_lpv(lpvdata, dep, gridsize, 0.001);

% generating tight polytopic representation
hull = 'snnn';
U = genhull(U, hull);
S = coretensor(U, lpvdata, dep);

% plot the results
plothull(U, domain);

% check model approximation error
[maxerr meanerr] = tperror(lpv, S, U, domain, 100);
disp('max and mean error:'); disp(maxerr); disp(meanerr);

save('tora_data', 'S', 'U', 'n', 'domain', 'gridsize');


P = size(domain,1);
A = cell(size(S,1:P));

% pass
for i = 1:size(S,1)
    for j = 1:size(S,2)
        
        A{i,j}(:,:) = S(i,j,:,:);
        
    end
end
