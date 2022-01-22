function A = setA(o)
%Calculate A matrix of linear Sys.
% Use tp model transformation to obtain A by a lpv Sys.

%% tunable parameters
domain = [-1 1; -1 1; -1 1; -1 1];
gridsize = [10 10 10 10];
SV_TOLERANCE = 0.001;

%% hosvd
num_p = 4; % length of parameter vector of lpv system (p = [x1, x2, x3, x4])
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

A = cell(size(S,1:size(domain,1)));
for i1 = 1:size(S,1)
    for i2 = 1:size(S,2)
        for i3 = 1:size(S,3)
            for i4 = 1 : size(S, 4)
                A{i1,i2,i3,i4}(:,:) = S(i1,i2,i3,i4,:,:);
            end
        end        
    end
end

end

