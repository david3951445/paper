% % old method
% function [B points] = setB(o)
% %Calculate B matrix of linear Sys.
% % Use tp model transformation to obtain B by a lpv Sys.
% % B         - B matrix
% % points    - discrete membership function of B

% %% tunable parameters
% domain = [-1 1; -1 1];
% gridsize = [10 10];
% SV_TOLERANCE = 0.001;

% %% hosvd
% num_p = length(gridsize); % length of parameter vector of lpv system (p = [x1,x3])
% lpv = o.Bl;
% dep = zeros([size(lpv) num_p]);
% dep(2,1,:) = [1 1];
% dep(2,2,:) = [1 1];
% dep(4,1,:) = [1 1];
% dep(4,2,:) = [1 1];

% lpvdata = sampling_lpv(lpv, dep, domain, gridsize);
% [S U sv tol] = hosvd_lpv(lpvdata, dep, gridsize, SV_TOLERANCE); % hosvd
% U = genhull(U, 'close'); % generating tight polytopic representation
% plothull(U, domain); % plot the results

% % check model approximation error
% % [maxerr meanerr] = tperror(lpv, S, U, domain, 100);
% % disp('max and mean error:'); disp(maxerr); disp(meanerr);

% dim = size(S);
% dimL2 = dim(length(dim) - 1); % last 2-nd dimension
% dimL1 = dim(length(dim)); % last 1-st dimension
% len2 = 1;
% for i = 1 : num_p
%     len2 = len2*dim(i);
% end
% B = cell(1, len2);
% for i = 1 : len2
%     B{i} = zeros(dimL2, dimL1);
%     for j = 1 : dimL2*dimL1
%         B{i}(j) = S(i + len2*(j-1));
%     end
% end



% % set discrete membership function (Use to construct CT membership function further)
% % xS = cell(size(xS_domain, 1), 1);
% % % sample x
% for i = 1 : num_p
%     points{i}.x = linspace(domain(i, 1),domain(i, 2), gridsize(i));
%     points{i}.y = U{i};
% end

% end

