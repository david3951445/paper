classdef TPmodel
    %TP type polytopic LPV model (transformed from LPV model)
    
    properties
        sizeO       % Origin size in hosvd result, just for indexing M.mf(). ex: size(S) = [9, 2, 9, 2, 4, 4] -> A.sizeO = size(S, 1 : 4)
        val         % Core tensor (or "linear matrice")
        len         % number of Core tensor (length(val))
        % index       % for indexing "mf_discrete"
    end

    properties (Access = private)
        mf_discrete % Discrete (because it's composed by "points") membership function (or "weighting functions") of Core tensor
        index       % for indexing "mf_discrete"
    end
    
    methods
        function M = TPmodel(lpvPara)
            %Transform "LPV model" to "TP type polytopic LPV model"
            % see https://en.wikipedia.org/wiki/TP_model_transformation_in_control_theory to better understand
            % - lpvPara : LPV system parameter
            
            domain          = lpvPara.domain;
            gridsize        = lpvPara.gridsize;
            SV_TOLERANCE    = lpvPara.SV_TOLERANCE;
            lpv             = lpvPara.val;
            num_p           = lpvPara.num_p;
            dep             = lpvPara.dep;

            lpvdata = sampling_lpv(lpv, dep, domain, gridsize);
            [S U sv tol] = hosvd_lpv(lpvdata, dep, gridsize, SV_TOLERANCE); % hosvd
            U = genhull(U, 'close'); % generating tight polytopic representation
            plothull(U, domain); % plot the results

            dim = size(S);
            % this two dimension contain the linear matrix
            dimL2 = dim(length(dim) - 1); % last 2-nd dimension
            dimL1 = dim(length(dim)); % last 1-st dimension

            %% set M.sizeO
            M.sizeO = dim(1 : num_p);

            %% set M.val by S
            % The way of indexing is changed here.
            % ex: S = [4, 2, 4, 2, :, :] -> A = {64}(:,:).
            len = prod(M.sizeO); % ex: prod([4 2 4 2]) = 64
            M.val = cell(1, len);
            for i = 1 : len
                M.val{i} = zeros(dimL2, dimL1);
                for j = 1 : dimL2*dimL1
                    M.val{i}(j) = S(i + len*(j-1));
                end
            end
            M.len = len;

            %% set discrete membership function (Use to construct CT membership function further)
            M.mf_discrete = cell(1, num_p);
            for i = 1 : num_p
                M.mf_discrete{i}.x = linspace(domain(i, 1),domain(i, 2), gridsize(i));
                M.mf_discrete{i}.y = U{i};
            end

            M.index = Combvec(M.sizeO);

            
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
        
        function y = mf(M, x, ind)
            %Calculate membership function of ind-th

            ind = M.index(:, ind);
            n = length(M.mf_discrete);         
            y = 1; % output of mf (value: 0~1)
            for i = 1 : n % multiply all "premise variable"
                p = x(i);
                X = M.mf_discrete{i}.x;
                Y = M.mf_discrete{i}.y;
                k = ind(i);
            
                m = length(X);
                if p < X(1)
                    y = y*Y(1, k);
                elseif p > X(n)
                    y = y*Y(n, k);
                else
                    for j = 2 : m
                        if p < X(j)
                            slope = (Y(j, k) - Y(j-1, k))/(X(j) - X(j-1));
                            % Y(j-1, k)
                            % slope*(p - X(j-1))
                            y = y*(Y(j-1, k) + slope*(p - X(j-1)));
                            break;
                        end
                    end
                end

            end   
        end
    end

    methods (Access = private)
        function S = saveobj(o)
            S = o;
        end
    end
end

