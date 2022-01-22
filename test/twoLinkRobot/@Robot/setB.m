function setB(o)
%Calculate B matrix of linear Sys.
% Use tp model transformation to obtain B by a lpv Sys.

%% tunable parameters
domain = [-1 1; -1 1; -1 1; -1 1];
gridsize = [10 10 10 10];
SV_TOLERANCE = 0.001;

%% hosvd
num_p = 2; % length of parameter vector of lpv system (p = [x1,x3])
lpv = o.Bl; 
dep = zeros([size(lpv) num_p]);
dep(2,1,:) = [1 0 1 0];
dep(2,2,:) = [1 0 1 0];
dep(4,1,:) = [1 0 1 0];
dep(4,2,:) = [1 0 1 0];

lpvdata = sampling_lpv(lpv, dep, domain, gridsize);
[S U sv tol] = hosvd_lpv(lpvdata, dep, gridsize, SV_TOLERANCE); % hosvd
U = genhull(U, 'close'); % generating tight polytopic representation
plothull(U, domain); % plot the results

% check model approximation error
% [maxerr meanerr] = tperror(lpv, S, U, domain, 100);
% disp('max and mean error:'); disp(maxerr); disp(meanerr);

o.B = cell(size(S,1:size(domain,1)));
for i1 = 1:size(S,1)
    for i2 = 1:size(S,2)
        o.B{i1,i2}(:,:) = S(i1,i2,:,:);      
    end
end

end

