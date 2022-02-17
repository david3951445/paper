function M = constructSymMatrix(M)
%Construct symmetric matrix
% Given a lower triangle cell matrix, fill it's remain portion
% ex.
%       input:  M = {1 2 3
%                    4 5 6
%                    7 8 9}
%       output: M = [1 2 3
%                    2 5 6
%                    3 5 9]

for i = 1 : size(M, 1)
    for j = i + 1 : size(M, 2)
        M{i, j} = M{j, i}';
    end
end

M = cell2mat(M);
end