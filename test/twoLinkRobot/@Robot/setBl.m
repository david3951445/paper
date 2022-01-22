function y = setBl(o, x, position) % Bl constructor
    m1 = o.m1; m2 = o.m2; l1 = o.l1; l2 = o.l2;
    s1 = sin(x(1)); s2 = sin(x(3)); c1 = cos(x(1)); c2 = cos(x(3));
    SSCC = (s1*s2+c1*c2);

    switch position
        case 21 % A21
            num = m2*l2^2;
        case 22
            num = -m2*l1*l2*SSCC;
        case 41
            num = -m2*l1*l2*SSCC;
        case 42
            num = ((m1+m2)*l1^2);
        otherwise
            error(['cannot construct B' str(position) ' please check your Bl matrix'])
    end

    den = m2*l1^2*l2^2*((m1+m2) - m2*SSCC^2);
    
    y = num/den;
end