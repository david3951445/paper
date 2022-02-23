function o = getTPmodel(o, para)
%Transform "LPV model" to "TP type polytopic LPV model"
% see https://en.wikipedia.org/wiki/TP_model_transformation_in_control_theory to better understand
% - lpvPara : LPV system parameter
domain          = para.domain;
gridsize        = para.gridsize;
SV_TOLERANCE    = para.SV_TOLERANCE;
lpv             = para.lpv;
num_p           = para.num_p;
dep             = para.dep;

lpvdata = sampling_lpv(lpv, dep, domain, gridsize);
[S U sv tol] = hosvd_lpv(lpvdata, dep, gridsize, SV_TOLERANCE); % hosvd
U = genhull(U, 'close'); % generating tight polytopic representation
plothull(U, domain); % plot the results

dim = size(S);
% this two dimension contain the linear matrix
dimL2 = dim(length(dim) - 1); % last 2-nd dimension
dimL1 = dim(length(dim)); % last 1-st dimension

%% set M.sizeO
o.sizeO = dim(1 : num_p);

%% set M.val by S
% The way of indexing is changed here.
% ex: S = [4, 2, 4, 2, :, :] -> A = {64}(:,:).
len = prod(o.sizeO); % ex: prod([4 2 4 2]) = 64
o.val = cell(1, len);
for i = 1 : len
    o.val{i} = zeros(dimL2, dimL1);
    for j = 1 : dimL2*dimL1
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

%% test if A{64} == A{4,2,4,2}
% disp('tpmodel')
% sum = 0;
% sum_A = zeros(4);
% for i = 1 : M.len
%     ind = M.index(:, i);
%     i1 = ind(1);i2 = ind(2);i3 = ind(3);i4 = ind(4);
%     A{i1,i2,i3,i4}(:,:) = S(i1,i2,i3,i4,:,:);
%     sum_A = sum_A + M.mf([0 0 0 0], i)*A{i1,i2,i3,i4};
% end
% disp(['sum of mbfun of A: ' num2str(sum)])
% disp('sum of of A: ')
% disp(sum_A)

end