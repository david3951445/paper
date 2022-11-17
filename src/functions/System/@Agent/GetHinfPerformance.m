%Calculate (x'Qx + u'Ru - V(0))/v'v

function y = GetHinfPerformance(ag)
    % num
    num = 0;
    for i = 1 : ag.tr.LEN
        xQ1x = WeightedNorm(ag.sys_aug.Q1, ag.tr.x(:, i));
        xQ2x = WeightedNorm(ag.sys_aug.Q2, ag.tr.xh(:, i));
        uRu = WeightedNorm(ag.sys_aug.R, ag.tr.u(:, i));
        num = num + xQ1x + xQ2x + uRu;
    end
    % V(0)
    num = num - WeightedNorm(ag.sys_aug.P1, ag.tr.x0) - WeightedNorm(ag.sys_aug.P2, ag.tr.xh0);
    % den
    den = 0;
    for i = 1 : ag.tr.LEN
        v = [ag.sys_a.fault(:, i); ag.sys_s.fault(:, i)];
        den = den + 2*norm(v);
    end

    y = num/den;
    num
    den
end