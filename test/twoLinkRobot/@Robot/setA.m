function [A, points] = setA(o)
%Calculate A matrix, membershio function of linear Sys.
% Use tp model transformation to obtain A by a lpv Sys.
% A         - A matrix
% points    - discrete membership function of A

%% tunable parameters
domain = [-1 1; -1 1; -1 1; -1 1];
gridsize = [10 10 10 10];
SV_TOLERANCE = 0.001;

%% hosvd
num_p = length(gridsize); % length of parameter vector of lpv system (p = [x1, x2, x3, x4])
lpv = o.Al; 
dep = zeros([size(lpv) num_p]);
dep(2,1,:) = [1 0 1 0];
dep(2,2,:) = [1 1 1 0];
dep(2,3,:) = [1 0 1 0];
dep(2,4,:) = [1 0 1 1];
dep(4,1,:) = [1 0 1 0];
dep(4,2,:) = [1 1 1 0];
dep(4,3,:) = [1 0 1 0];
dep(4,4,:) = [1 0 1 1];

lpvdata = sampling_lpv(lpv, dep, domain, gridsize);
[S U sv tol] = hosvd_lpv(lpvdata, dep, gridsize, SV_TOLERANCE); % hosvd
U = genhull(U, 'close'); % generating tight polytopic representation
plothull(U, domain); % plot the results

% check model approximation error
% [maxerr meanerr] = tperror(lpv, S, U, domain, 100);
% disp('max and mean error:'); disp(maxerr); disp(meanerr);

% index = Combvec(size(S, 1:4));
% A = cell(1, length(index));
% for i = 1 : length(index)
%     A{i}(:, :) = S(i1,i2,i3,i4,:,:)
% end

dim = size(S);
dimL2 = dim(length(dim) - 1); % last 2-nd dimension
dimL1 = dim(length(dim)); % last 1-st dimension
len2 = 1;
for i = 1 : num_p
    len2 = len2*dim(i);
end
A = cell(1, len2);
for i = 1 : len2
    A{i} = zeros(dimL2, dimL1);
    for j = 1 : dimL2*dimL1
        A{i}(j) = S(i + len2*(j-1));
    end
end

% old method to obtain A
% A = cell(size(S, 1 : num_p));
% for i1 = 1:size(S, 1)
%     for i2 = 1:size(S, 2)
%         for i3 = 1:size(S, 3)
%             for i4 = 1 : size(S, 4)
%                 A{i1, i2, i3, i4}(:,:) = S(i1,i2,i3,i4,:,:)
%             end
%         end        
%     end
% end

% set discrete membership function (Use to construct CT membership function further)
% xS = cell(size(xS_domain, 1), 1);
% % sample x
points = cell(1, num_p);
for i = 1 : num_p
    points{i}.x = linspace(domain(i, 1),domain(i, 2), gridsize(i));
    points{i}.y = U{i};
end

end