% % old method
% function M = TPmodelTransf(lpvPara)
% %Transform "LPV model" to "TP type polytopic LPV model"
% % see https://en.wikipedia.org/wiki/TP_model_transformation_in_control_theory to better understand
% % - lpvPara : LPV system parameter
% % - type      : what's part do u want to tansform?
% %               dxdt = f(x)x + g(x)u -> dxdt = Ax + Bu
% % - M         : TP type polytopic transformed from LPV system
% %     - M.val           : Core tensor (or "linear matrice")
% %     - M.mf_discrete   : Discrete (because it's composed by "points") membership function (or "weighting functions") of Core tensor 
% %     - M.sizeO         : Origin size in hosvd result, just for indexing A.mf
% %                         ex. size(S) = [9, 2, 9, 2, 4, 4] -> A.sizeO = size(S, 1 : 4)                  

% domain          = lpvPara.domain;
% gridsize        = lpvPara.gridsize;
% SV_TOLERANCE    = lpvPara.SV_TOLERANCE;
% lpv             = lpvPara.val;
% num_p           = lpvPara.num_p;
% dep             = lpvPara.dep;

% lpvdata = sampling_lpv(lpv, dep, domain, gridsize);
% [S U sv tol] = hosvd_lpv(lpvdata, dep, gridsize, SV_TOLERANCE); % hosvd
% U = genhull(U, 'close'); % generating tight polytopic representation
% plothull(U, domain); % plot the results

% % check model approximation error
% % [maxerr meanerr] = tperror(lpv, S, U, domain, 100);
% % disp('max and mean error:'); disp(maxerr); disp(meanerr);

% dim = size(S);
% % this two dimension contain the linear matrix
% dimL2 = dim(length(dim) - 1); % last 2-nd dimension
% dimL1 = dim(length(dim)); % last 1-st dimension

% %% set M.sizeO
% M.sizeO = dim(1 : num_p);

% %% set M.val by S
% % The way of indexing is changed here.
% % ex: S = [4, 2, 4, 2, :, :] -> A = {4*2*4*2}(:,:)
% len = prod(M.sizeO); % ex: prod([1 2 3]) = 1*2*3
% M.val = cell(1, len);
% for i = 1 : len
%     M.val{i} = zeros(dimL2, dimL1);
%     for j = 1 : dimL2*dimL1
%         M.val{i}(j) = S(i + len*(j-1));
%     end
% end

% %% set discrete membership function (Use to construct CT membership function further)
% M.mf_discrete = cell(1, num_p);
% for i = 1 : num_p
%     M.mf_discrete{i}.x = linspace(domain(i, 1),domain(i, 2), gridsize(i));
%     M.mf_discrete{i}.y = U{i};
% end

% end


% % old method to obtain A and B
% % A = cell(size(S, 1 : num_p));
% % for i1 = 1:size(S, 1)
% %     for i2 = 1:size(S, 2)
% %         for i3 = 1:size(S, 3)
% %             for i4 = 1 : size(S, 4)
% %                 A{i1, i2, i3, i4}(:,:) = S(i1,i2,i3,i4,:,:)
% %             end
% %         end        
% %     end
% % end
% %
% % B = cell(size(S,1:size(domain,1)));
% % for i1 = 1:size(S,1)
% %     for i2 = 1:size(S,2)
% %         B{i1,i2}(:,:) = S(i1,i2,:,:);      
% %     end
% % end