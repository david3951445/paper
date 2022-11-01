function o = getTPmodel(o, para)
%Transform "LPV model" to "TP type polytopic LPV model"
%
% - how to tansform nonlinear system to LPV system
% https://www.researchgate.net/publication/347277203_Study_of_tensor_product_model_alternatives
% - wiki's introduction
% https://en.wikipedia.org/wiki/TP_model_transformation_in_control_theory


%% - lpvPara : LPV system parameter
domain          = para.domain;
gridsize        = para.gridsize;
SV_TOLERANCE    = para.SV_TOLERANCE;
lpv             = para.lpv;
num_p           = para.num_p;
dep             = para.dep;

lpvdata = sampling_lpv(lpv, dep, domain, gridsize);
[S U sv tol] = hosvd_lpv(lpvdata, dep, gridsize, SV_TOLERANCE); % hosvd
U = genhull(U, 'close'); % generating tight polytopic representation

%% set o.sizeO
dim = size(S);
o.sizeO = dim(1 : num_p);
DIM_L =  dim(length(dim) - 1 : length(dim)); % last 2 dimension (this two dimension contain the linear matrix)

%% set o.val, o.len
% The way of indexing the linear matrix is changed here.
% ex: S = [4, 2, 4, 2, :, :] -> A = {64}(:,:).
len = prod(o.sizeO); % ex: prod([4 2 4 2]) = 64
o.val = cell(1, len);
for i = 1 : len
    o.val{i} = zeros(DIM_L);
    for j = 1 : prod(DIM_L)
        o.val{i}(j) = S(i + len*(j-1));
    end
end
o.len = len;

%% set discrete membership function (Use to construct CT membership function further)
o.mf_discrete = cell(1, num_p);
for i = 1 : num_p
    o.mf_discrete{i}.x = linspace(domain(i, 1),domain(i, 2), gridsize(i));
    o.mf_discrete{i}.y = U{i};
end

o.index = Combvec(o.sizeO);

%% If you want to check model approximation error:
% [maxerr meanerr] = tperror(lpv, S, U, domain, len);
% disp('max and mean error:'); disp(maxerr); disp(meanerr);

%% If you want to plot interpolation function
plothull(U, domain); % plot the results

%% Check sum of A{4,2,4,2} and corresponding interpolation function (test if changing index success, i.e., A{4,2,4,2} = A{64})
% sum_A = zeros(DIM_L);
% testPoint = rand(DIM_L(1)); % The test point for interpolation Function

% for i = 1 : o.len
%     ind = o.index(:, i);
%     h = o.mf(testPoint, i);
%     i1 = ind(1);i2 = ind(2);i3 = ind(3);
%     A{i1,i2,i3}(:,:) = S(i1,i2,i3,:,:);
%     sum_A = sum_A + h*A{i1,i2,i3};
%     disp(['norm of A{i1,i2,i3}: ' num2str(norm(A{i1,i2,i3}))])
%     disp(['interpolation function of A{i1,i2,i3}: ' num2str(h)])
% end

% disp('sum of origin A: ')
% disp(sum_A)

end