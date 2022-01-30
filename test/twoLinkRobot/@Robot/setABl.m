function y = setABl(o, p, position)
%ABl constructor
% parameter vector in lpv model : p = [x1 x2 x3 x4 sin(x1)/x1 sin(x3)/x3]

m1 = o.m1; m2 = o.m2; l1 = o.l1; l2 = o.l2; g  = o.g;

% s1x1 = p(5); % sin(x1)/x1
% s2x3 = p(6); % sin(x3)/x3
s1 = sin(p(1));
s2 = sin(p(3));
c1 = cos(p(1));
c2 = cos(p(3));
s1x1 = s1/p(1); % sin(x1)/x1
s2x3 = s2/p(3); % sin(x3)/x3
SSCC = (s1*s2+c1*c2);
SCSC = (s1*c2-c1*s2);

% Numerator
switch position
    % A
    case 21 % A21
        num = (m1+m2)*l2*g*s1x1;
    case 22
        num = SCSC*m2*l1*l2*SSCC*p(2);
    case 23
        num = -m2*l2*g*SSCC;
    case 24
        num = -SCSC*m2*l2^2*p(4);
    case 41
        num = -(m1+m2)*l1*g*SSCC*s1x1;
    case 42
        num = -SCSC*(m1+m2)*l1^2*p(2);
    case 43
        num = (m1+m2)*l1*g*s2x3;
    case 44
        num =  SCSC*m2*l1*l2*SSCC*p(4);
    
    % B
    case 25
        num = m2*l2^2;
    case 26
        num = -m2*l1*l2*SSCC;
    case 45
        num = -m2*l1*l2*SSCC;
    case 46
        num = (m1+m2)*l1^2;

    otherwise
        error(['cannot construct AB' str(position) ' please check your ABl matrix'])
end

% denominator
switch position
    % A
    case {21, 22, 23, 24}
        den = l1*l2*((m1+m2) - m2*SSCC^2);
    case {41, 42, 43, 44}
        den = (m1+m2)*l2*g*s1 - m2*l2*g*s2*SSCC;
    
    % B
    case {25, 26, 45, 46}
        den = m2*l1^2*l2^2*((m1+m2) - m2*SSCC^2);
end

y = num/den;

end