% LMI part
function m = getControlGain2(uav, fz, ref, p, m)
    I = eye(uav.dim);
%     m.K = zeros(uav.dim_u, uav.dim, fz.num);
    
    for i = 1 : fz.num
        A = uav.A(:, :, i); B = uav.B(:, :, i);

       %% find P1 K
        setlmis([]);

        % var
        W1 = lmivar(1, [uav.dim 1]); % dim x dim
        Y1 = lmivar(2, [uav.dim uav.dim_u]); % dim x dim_u
        P2 = lmivar(1, [uav.dim 1]); % dim x dim

        % LMIs
        lmiterm([-1 1 1 W1], 1, 1); % W1>0
        lmiterm([-3 1 1 P2], 1, 1); % P2>0
        
        lmiterm([2 1 1 W1], 1, A', 's');
        lmiterm([2 1 1 Y1], 1, B', 's');
        lmiterm([2 1 1 0], I/p.rho^2);
        lmiterm([2 1 2 W1], -1, p.Q); % ignore term K
        lmiterm([2 2 2 0], p.Q);
        lmiterm([2 2 2 P2], ref.A', 1, 's');
        lmiterm([2 1 3 W1], 1, sqrt(p.Q));
        lmiterm([2 3 3 0], -I);
        lmiterm([2 2 4 P2], 1, 1);
        lmiterm([2 4 4 0], -p.rho^2*I/(ref.B*ref.B'));

        % solve
        lmis = getlmis;
        [~, xfeas] = feasp(lmis);
        W1 = dec2mat(lmis, xfeas, W1);
        Y1 = dec2mat(lmis, xfeas, Y1);
        P2 = dec2mat(lmis, xfeas, P2);

        P1 = I/W1;
        K = (P1*Y1)';
        M11 = p.Q + symmetric(A'*P1 + K'*B'*P1) + P1*P1/p.rho^2;
        
%         max(eig(P1))
%         max(eig(P2))
%         max(eig(M11))
        
        m.P1(:, :, i) = P1;
        m.P2(:, :, i) = P2;
        m.K(:, :, i) = K;
    end
    
    %% function
    function y = symmetric(x)
    y = x + x';
    end
end