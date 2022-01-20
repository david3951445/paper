clc; clear;
M{1, 1} = eye(2);
M{2, 1} = [1 1];
M{2, 2} = 3;
M = fillUpperTriangle(M);
function M = fillUpperTriangle(M)
    for i = 1 : size(M, 1)
        for j = i + 1 : size(M, 2)
            M{i, j} = M{j, i}';
        end
    end
    M = cell2mat(M);
end