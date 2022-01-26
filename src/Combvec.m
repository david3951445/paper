function y = Combvec(sizes)
%Like combvec in MATLAB
% ex.
%      input    : sizes = [2, 3] 
%      output   : [1     1     1     2     2     2
%                  1     2     3     1     2     3]
y = backtracking([], [], sizes, 1);

    function y = backtracking(y, cur, sizes, index)
        if (index == length(sizes) + 1)
            y = cat(2, y, cur);
            return;
        end
        
        for i = 1 : sizes(index)
            y = backtracking(y, cat(1, cur, i), sizes, index + 1);
        end
    end
end

