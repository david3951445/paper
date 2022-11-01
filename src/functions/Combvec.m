%Like combvec in MATLAB
% ex.
%      input    : sizes = [2, 3] 
%      output   : [1     1     1     2     2     2
%                  1     2     3     1     2     3]

function y = Combvec(sizes)
y = backtracking([], [], sizes, length(sizes));

    function y = backtracking(y, cur, sizes, index)
        if (index == 0)
            y = cat(2, cur, y);
            return;
        end
        
        for i = sizes(index) : -1 : 1
            y = backtracking(y, cat(1, i, cur), sizes, index - 1);
        end
    end
end

