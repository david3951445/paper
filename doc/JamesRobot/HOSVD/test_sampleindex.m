clc;clear;close all
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
epsilon = 1;
% functions in the system matrix
F = @(p) sin(epsilon * p(1));
H = @(p) p(2);
% system matrix: lpv = [A(p) B(p)]
lpv = {...
    @(p)0         @(p)1   ; 
    @(p)H(p)      @(p)F(p);
    };

% parameter dependencies:
% dep(i,j,k) is 1 if Sp{i,j} depends on p(k)
dep = zeros([size(lpv) 2]);
dep(:,:,1) = [0 0; 0 1];
dep(:,:,2) = [0 0; 1 0];
% sampling intervals for each parameter
domain = [0 1; 0 1];
% grid size: number of grid points for each parameter
gridsize = [11 11];

%% TP transformation, same as:
%   [S U] = tptrans(lpv, dep, domain, gridsize, 'close');

% sampling
lpvdata = sampling_lpv(lpv, dep, domain, gridsize);

% hosvd
[S U sv tol] = hosvd_lpv(lpvdata, dep, gridsize);

% % generating tight polytopic representation
hull = 'close';
U = genhull(U, hull);
S = coretensor(U, lpvdata, dep);
% 
% plot the results
plothull(U, domain);
% 
% check model approximation error
[maxerr meanerr] = tperror(lpv, S, U, domain, 100);
disp('max and mean error:'); disp(maxerr); disp(meanerr);
% 



P = size(domain,1);
A = cell(size(S,1:P));

% pass
for i = 1:size(S,1)
    for j = 1:size(S,2)
        
        A{i,j}(:,:) = S(i,j,:,:);
        
    end
end

AA = U{1}(8,1)*U{2}(5,1)*A{1,1} + U{1}(8,1)*U{2}(5,2)*A{1,2} + ...
    U{1}(8,2)*U{2}(5,1)*A{2,1} + U{1}(8,2)*U{2}(5,2)*A{2,2}
%%
xS_domain = domain;
xS_gridsize = gridsize;
xS = cell(size(xS_domain,1),1);
% sample x
for i = 1:size(xS_domain,1)
    xS{i} = linspace(xS_domain(i,1),xS_domain(i,2),xS_gridsize(i));
end
x = [0.7 0.4];
H = mf(x,xS,U);


c = 1;
for i = 1:size(S,1)
    for j = 1:size(S,2)
        Abar(:,:,c) = A{i,j}(:,:);
        c = c+1;
    end
end







AAA = defuzzy(H,Abar)
function H = mf(x,xS,U)
    H = cell(1,size(U,2));
    n = zeros(1,size(U,2)); 
    for i = 1:size(U,2)
        n(i) = size(U{i},1); % number of sample point of each premise variable
    end
    
    for i = 1:size(U,2) % number of premise variable
        for j = 1:n(i)
            if x(i) <= xS{i}(1)
                H{i} = U{i}(1,:);
                break;
            elseif x(i) <= xS{i}(j)
                h1 = (x(i)-xS{i}(j-1))/(xS{i}(j)-xS{i}(j-1));
                h2 = 1-h1;
                %%% 分點公式 %%%
                H{i} = h1*U{i}(j,:) + h2*U{i}(j-1,:);
                break;
            elseif x(i) > xS{i}(n(i))
                H{i} = U{i}(n(i),:);
                break;
            end
        end
    end
end

function Kbar = defuzzy(H,K)
c = 1;
Kbar = 0; 
for i1 = 1:size(H{1},2)
    for i2 = 1:size(H{2},2)
                h(c) = H{1}(i1)*H{2}(i2);
                c = c+1;
    end
end

for i = 1:c-1
    Kbar = h(i)*K(:,:,i) + Kbar;
end

end
