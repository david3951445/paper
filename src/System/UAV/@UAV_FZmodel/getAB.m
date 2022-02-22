function obj = getAB(obj, fz)  
    s = sym('s', [obj.DIM_X, 1]);
    
    % find A
    obj.A = cell(1, fz.num);
    disp('Construct A, B matrix...');
    for k = 1 : fz.num
        disp(['A' num2str(k) ' constructed'])
                  
        y = subs(obj.f(s), s(fz.PV), fz.set(:, k)); % Sub fuzzy operation point into nonlinear function f()
        
        % We want f(x) = [f1 ... fn]' = A*x = [A1 ... An]'*[x1 ... xn]', where A1 is row 1 of A
        obj.A{k} = zeros(obj.DIM_X);
        for i = 1 : obj.DIM_X % i-th row of A               
            [cf, u] = coeffs(y(i)); % ex. f1 = s2 + 2*s3 -> cf = [1 2], u = [s2, s3]
            u = subs(u, s, (1:obj.DIM_X)'); % u = [s2, s3] -> u = [2, 3]
            obj.A{k}(i, u) = cf; % A1 = [0 1 1 0 ... 0]
        end
        obj.A{k};
    end

    % find B
    obj.B = cell(1, fz.num);
    for k = 1 : fz.num
        disp(['B' num2str(k) ' constructed'])
                    
        y = subs(obj.g(s), s(fz.PV), fz.set(:, k));

        obj.B{k} = zeros(obj.DIM_X, obj.DIM_U);
        obj.B{k}(:,:) = subs(y, s(11), 0); % let phi = 0
    end
end