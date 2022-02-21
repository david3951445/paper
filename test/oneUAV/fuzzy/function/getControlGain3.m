% LMI part % method 2
function m = getControlGain3(uav, fz, ref, p, m)
    I = eye(uav.dim); O = zeros(uav.dim);
    I2 = eye(uav.dim*2);
%     m.K = zeros(uav.dim_u, uav.dim, fz.num);
    
    for i = 1 : fz.num
        A = uav.A(:, :, i); B = uav.B(:, :, i);
        Ab = [A O;-I O]; Bb = [B; zeros(uav.dim, uav.dim_u)];
        Qb = [O O; O p.Q];
       %% find P1 K
        setlmis([]);

        % var
        W = lmivar(1, [uav.dim*2 1]); 
        Y = lmivar(2, [uav.dim_u uav.dim*2]);

        % LMIs
        lmiterm([-1 1 1 W], 1, 1); % W>0
        
        lmiterm([2 1 1 W], Ab, 1, 's');
        lmiterm([2 1 1 Y], Bb, 1, 's');
        lmiterm([2 1 1 0], I2/p.rho^2);
        lmiterm([2 1 2 W], 1, sqrt(Qb));
        lmiterm([2 2 2 0], -I2);

        % solve
        lmis = getlmis;
        [~, xfeas] = feasp(lmis);
        W = dec2mat(lmis, xfeas, W);
        Y = dec2mat(lmis, xfeas, Y);

        P = I2/W;
        K = Y*P;
        
%         max(eig(P1))
%         max(eig(P2))
%         max(eig(M11))
        
        m.P(:, :, i) = P;
        m.K(:, :, i) = K;
    end
    
    %% function
    function y = symmetric(x)
    y = x + x';
    end
end