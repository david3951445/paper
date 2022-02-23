function y = mf(o, x, ind)
    %Calculate membership function of ind-th

    ind = o.index(:, ind);
    n = length(o.mf_discrete);         
    y = 1; % output of mf (value: 0~1)
    for i = 1 : n % multiply all "premise variable"
        p = x(i);
        X = o.mf_discrete{i}.x;
        Y = o.mf_discrete{i}.y;
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