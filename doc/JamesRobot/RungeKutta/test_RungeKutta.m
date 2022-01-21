clc;clear;


LP1 = load('lpv_data_part1.mat');
LP2 = load('lpv_data_part2.mat');
LP3 = load('lpv_data_part3.mat');
% load ('gain.mat');

xS_domain = [LP1.domain;LP2.domain;LP3.domain];
xS_gridsize = [LP1.gridsize,LP2.gridsize,LP3.gridsize];
xS = cell(size(xS_domain,1),1);
% sample x
for i = 1:size(xS_domain,1)
    xS{i} = linspace(xS_domain(i,1),xS_domain(i,2),xS_gridsize(i));
end
% initialization
% continue ...




function M = mf(x,xS,U)
    n = size(U,1);
    for i = 1:n
        if x < xS(1)
            M = U(1);
            break;
        elseif x < xS(i)
            h1 = (x-xS(i-1))/(xS(i)-xS(i-1));
            h2 = 1-h1;
            %%% 分點公式 %%%
            M = h1*U(i) + h2*U(i-1);
            break;
        elseif x > xS(n) 
            M = U(n);
            break; 
        end
    end
end
