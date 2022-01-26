function M = TPmodelTransf(sysMatrix, type)
%Transform "LPV model" to "TP type polytopic LPV model"
% see https://en.wikipedia.org/wiki/TP_model_transformation_in_control_theory to better understand
% - sysMatrix : system matrix
% - type      : what's part do u want to tansform?
%               dxdt = f(x)x + g(x)u -> dxdt = Ax + Bu
% - M         : TP type polytopic transformed from system matrix
%     - M.val     : Core tensor (or "linear matrice")
%     - M.mf      : Discrete (because it's composed by "points") membership function (or "weighting functions") of Core tensor 
%     - M.sizeO   : Origin size in hosvd result, just for indexing A.mf
%                   ex. size(S) = [9, 2, 9, 2, 4, 4] -> A.sizeO = size(S, 1 : 4)                  

switch type
    %% tunable parameters
    case 'A'
        domain = [-1 1; -1 1; -1 1; -1 1];
        gridsize = [10 10 10 10];
        SV_TOLERANCE = 0.001;
    case 'B'
        domain = [-1 1; -1 1];
        gridsize = [10 10];
        SV_TOLERANCE = 0.001;
    otherwise
        error('no this type in setLinearSys()')
end

lpv = sysMatrix;
num_p = length(gridsize); % length of parameter vector of lpv system (p = [x1, x2, x3, x4])
dep = zeros([size(lpv) num_p]);

%% hosvd
switch type
    % dep setting
    case 'A'
        dep(2,1,:) = [1 0 1 0];
        dep(2,2,:) = [1 1 1 0];
        dep(2,3,:) = [1 0 1 0];
        dep(2,4,:) = [1 0 1 1];
        dep(4,1,:) = [1 0 1 0];
        dep(4,2,:) = [1 1 1 0];
        dep(4,3,:) = [1 0 1 0];
        dep(4,4,:) = [1 0 1 1];
    case 'B'
        dep(2,1,:) = [1 1];
        dep(2,2,:) = [1 1];
        dep(4,1,:) = [1 1];
        dep(4,2,:) = [1 1];
end

lpvdata = sampling_lpv(lpv, dep, domain, gridsize);
[S U sv tol] = hosvd_lpv(lpvdata, dep, gridsize, SV_TOLERANCE); % hosvd
U = genhull(U, 'close'); % generating tight polytopic representation
plothull(U, domain); % plot the results

% check model approximation error
% [maxerr meanerr] = tperror(lpv, S, U, domain, 100);
% disp('max and mean error:'); disp(maxerr); disp(meanerr);

dim = size(S);
dimL2 = dim(length(dim) - 1); % last 2-nd dimension
dimL1 = dim(length(dim)); % last 1-st dimension
len2 = 1;
for i = 1 : num_p
    len2 = len2*dim(i);
end
M.val = cell(1, len2);
for i = 1 : len2
    M.val{i} = zeros(dimL2, dimL1);
    for j = 1 : dimL2*dimL1
        M.val{i}(j) = S(i + len2*(j-1));
    end
end

% set discrete membership function (Use to construct CT membership function further)
% xS = cell(size(xS_domain, 1), 1);
% % sample x
M.mf = cell(1, num_p);
for i = 1 : num_p
    M.mf{i}.x = linspace(domain(i, 1),domain(i, 2), gridsize(i));
    M.mf{i}.y = U{i};
end

% set A.sizeO
M.sizeO = dim(1:num_p);

end

% old method to obtain A and B
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
%
% B = cell(size(S,1:size(domain,1)));
% for i1 = 1:size(S,1)
%     for i2 = 1:size(S,2)
%         B{i1,i2}(:,:) = S(i1,i2,:,:);      
%     end
% end