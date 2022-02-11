function [A, B] = getLocalMatrix(uav, fz)  
    s = sym('s', [uav.dim, 1]);
    
    % find A
    A = zeros(uav.dim, uav.dim, fz.num);
    for k = 1 : fz.num       
        y = subs(uav.f(s), s(fz.PV), fz.set(:, k));
        for i = 1 : uav.dim
            [cf, u] = coeffs(y(i));
            u = subs(u, s, (1:uav.dim)');
            A(i, u, k) = cf;
        end
    end

    % find B
    B = zeros(uav.dim, uav.dim_u, fz.num);
    for k = 1 : fz.num
        y = subs(uav.g(s), s(fz.PV), fz.set(:, k));
        B(:, :, k) = subs(y, s(11), 0); % let phi = 0
    end
end