function checkLinearizedSum(M, f)
%Display sum of Matrices of Linearized system and sum of its interpolation Function
% M     : Cell array, contains Matrices
% f     : interpolation Function
sum = 0;
sum_M = zeros(size(M{1}));
testPoint = randn(size(M{1}, 1)); % The test point for interpolation Function

for i = 1 : length(M)   
    h = f(testPoint, i);
    sum = sum + h;
    sum_M = sum_M + h.*M{i};
    disp(['norm of M{i}: ' num2str(norm(M{i}))])
    disp(['interpolation function of M{i}: ' num2str(h)])
end

disp(['sum of interpolation function: ' num2str(sum)])
disp('sum of Matrices: ')
disp(sum_M)
end