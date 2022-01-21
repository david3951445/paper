function M = constructSymMatrix(M)
%Construct symmetric matrix
% Given a lower triangle cell matrix, fill it's remain portion

for i = 1 : size(M, 1)
    for j = i + 1 : size(M, 2)
        M{i, j} = M{j, i}';
    end
end

M = cell2mat(M);
end

