classdef Robot
%robot model

properties (Constant, Access = private)
    m1 = 5, m2 = 3, l1 = 1, l2 = 2, g = 9.8;
end

properties
    A  % A matrix of linear systems
    Al % A matrix of lpv system
end

methods
    function o = Robot()           
        o.Al = {
            @(x)0               @(x)1               @(x)0               @(x)0
            @(x) o.Al_(x, 21)   @(x) o.Al_(x, 22)   @(x) o.Al_(x, 23)   @(x) o.Al_(x, 24)
            @(x)0               @(x)0               @(x)1               @(x)0
            @(x) o.Al_(x, 41)   @(x) o.Al_(x, 42)   @(x) o.Al_(x, 43)   @(x) o.Al_(x, 44)
        };
    end

    function S = saveobj(o)
        S = o;
    end

    function y = f(o, x, u) % dx/dt
        y = [x(2)
                o.f1(x)+[o.g11(x) o.g12(x)]*u
                x(4)
                o.f2(x)+[o.g12(x) o.g22(x)]*u];
    end

    function y = f1(o, x)
        m1 = o.m1;
        m2 = o.m2;
        l1 = o.l1;
        l2 = o.l2;
        g  = o.g;
        
        s1 = sin(x(1)); s2 = sin(x(3));
        c1 = cos(x(1)); c2 = cos(x(3));
        SSCC = (s1*s2+c1*c2);
        
        num1 = (s1*c2-c1*s2)*(m2*l1*l2*SSCC*x(2)^2 - m2*(l2*x(4))^2);
        num2 = (m1+m2)*l2*g*s1 - m2*l2*g*s2*SSCC;
        den = l1*l2*((m1+m2) - m2*SSCC^2);
        y = (num1 + num2)/den;
    end
    
    function y = f2(o, x)
        m1 = o.m1; 
        m2 = o.m2;
        l1 = o.l1;
        l2 = o.l2;
        g  = o.g;
        
        s1 = sin(x(1)); s2 = sin(x(3));
        c1 = cos(x(1)); c2 = cos(x(3));
        SSCC = (s1*s2+c1*c2);
        
        num1 = (s1*c2-c1*s2)*(-(m1+m2)*(l1*x(2))^2 + m2*l1*l2*SSCC*x(4)^2);
        num2 = -(m1+m2)*l1*g*s1*SSCC + (m1+m2)*l1*g*s2;
        den = (m1+m2)*l2*g*s1 - m2*l2*g*s2*SSCC;
        y = (num1 + num2)/den;
    end
    
    function y = g11(o, x)
        m1 = o.m1; 
        m2 = o.m2;
        l1 = o.l1;

        s1 = sin(x(1)); s2 = sin(x(3));
        c1 = cos(x(1)); c2 = cos(x(3));
        SSCC = (s1*s2+c1*c2);
        
        den =  l1^2*((m1+m2) - m2*SSCC^2);
        y = 1/den;
    end
    
    function y = g12(o, x) % g12=g22
        m1 = o.m1; 
        m2 = o.m2;
        l1 = o.l1;
        l2 = o.l2;

        s1 = sin(x(1)); s2 = sin(x(3));
        c1 = cos(x(1)); c2 = cos(x(3));
        SSCC = (s1*s2+c1*c2);
        
        num = -SSCC;
        den =  l1*l2*((m1+m2) - m2*SSCC^2);
        y = num/den;
    end
    
    function y = g22(o, x)
        m1 = o.m1; 
        m2 = o.m2;
        l2 = o.l2;

        s1 = sin(x(1)); s2 = sin(x(3));
        c1 = cos(x(1)); c2 = cos(x(3));
        SSCC = (s1*s2+c1*c2);

        num = m1+m2;
        den =  m2*l2^2*((m1+m2) - m2*SSCC^2);
        y = num/den;
    end   
end

methods (Access = private)
    function y = Al_(o, x, position) % Al constructor
        m1 = o.m1; m2 = o.m2; l1 = o.l1; l2 = o.l2; g  = o.g;
        s1 = sin(x(1)); s2 = sin(x(3)); c1 = cos(x(1)); c2 = cos(x(3));
        SSCC = (s1*s2+c1*c2); SCSC = (s1*c2-c1*s2);

        switch position
            case 21 % A21
                num = (m1+m2)*l2*g*s1/x(1);
            case 22
                num = SCSC*m2*l1*l2*SSCC*x(2);
            case 23
                num = -m2*l2*g*s2*SSCC/x(3);
            case 24
                num = -SCSC*m2*l2^2*x(4);
            case 41
                num = -(m1+m2)*l1*g*s1*SSCC/x(1);
            case 42
                num = -SCSC*(m1+m2)*l1^2*x(2);
            case 43
                num = (m1+m2)*l1*g*s2/x(3);
            case 44
                num =  SCSC*m2*l1*l2*SSCC*x(4);
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
end

end
%#ok<*PROPLC>