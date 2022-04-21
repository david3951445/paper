function y = m(rb, i)
%Just a mapping between m and MASS
switch i
    case 0
        y = rb.MASS(1);
    case {1, 2}
        y = rb.MASS(2);
    case {3, 4}
        y = rb.MASS(3);
    case {5, 6}
        y = rb.MASS(4);
    case {7, 8}
        y = rb.MASS(5);
    case {9, 10}
        y = rb.MASS(6);
    case {11, 12}
        y = rb.MASS(7);
end
end