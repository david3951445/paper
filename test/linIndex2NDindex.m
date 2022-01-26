% test for index type changing
clc; clear;
dim = [5 4 3];
len = 1;
for i = 1 : length(dim)
    len = len*dim(i);
end

%% 1, 2, 3, ... -> (1, 1), (2, 1), ...
 
for i = 1 : len
    d1 = mod(i-1, dim(1)) + 1;
    I1 = (i-d1)/dim(1) + 1;
    d2 = mod(I1-1, dim(2)) + 1;
    I2 = (I1-d2)/dim(2) + 1;
    d3 = (I2-1) + 1;
    
    disp(['(' num2str(d1) ',' num2str(d2) ',' num2str(d3) ')']);
end

%% (1, 1), (2, 1), ... -> 1, 2, 3, ...
% for d3 = 1 : dim(3)
%     for d2 = 1 : dim(2)
%         for d1 = 1 : dim(1)
%             a1 = d1-1;
%             a2 = dim(1)*(d2-1);
%             a3 = dim(1)*dim(2)*(d3-1);
%             i = 1 + a1+a2+a3
%         end
%     end
% end