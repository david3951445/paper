function y = setAl(o, p, position) % Al constructor
    m1 = o.m1; m2 = o.m2; l1 = o.l1; l2 = o.l2; g  = o.g;
    s1 = sin(p(1)); s2 = sin(p(3)); c1 = cos(p(1)); c2 = cos(p(3));
    SSCC = (s1*s2+c1*c2); SCSC = (s1*c2-c1*s2);

    switch position
        case 21 % A21
            num = (m1+m2)*l2*g*s1/p(1);
        case 22
            num = SCSC*m2*l1*l2*SSCC*p(2);
        case 23
            num = -m2*l2*g*s2*SSCC/p(3);
        case 24
            num = -SCSC*m2*l2^2*p(4);
        case 41
            num = -(m1+m2)*l1*g*s1*SSCC/p(1);
        case 42
            num = -SCSC*(m1+m2)*l1^2*p(2);
        case 43
            num = (m1+m2)*l1*g*s2/p(3);
        case 44
            num =  SCSC*m2*l1*l2*SSCC*p(4);
        otherwise
            error(['cannot construct A' str(position) ' please check your Al matrix'])
    end

    switch position
        case {21, 22, 23, 24}
            den = l1*l2*((m1+m2) - m2*SSCC^2);
        case {41, 42, 43, 44}
            den = (m1+m2)*l2*g*s1 - m2*l2*g*s2*SSCC;
    end
    
    y = num/den;
end