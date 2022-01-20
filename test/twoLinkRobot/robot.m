classdef robot
    %robot model
    
    properties (Constant)
        m1 = 5; m2 = 3; l1 = 1; l2 = 2; g = 9.8;
    end
    
    methods
        function obj = robot()
        end

        function y = f(obj, x, u) % dx/dt
            y = [x(2)
                 obj.f1(x)+[obj.g11(x) obj.g12(x)]*u
                 x(4)
                 obj.f2(x)+[obj.g12(x) obj.g22(x)]*u];
        end

        function y = f1(obj, x)
            m1 = obj.m1;
            m2 = obj.m2;
            l1 = obj.l1;
            l2 = obj.l2;
            g  = obj.g;
            
            s1 = sin(x(1)); s2 = sin(x(3));
            c1 = cos(x(1)); c2 = cos(x(3));
            SC = (s1*s2+c1*c2);
            
            num1 = (s1*c2-c1*s2)*(m2*l1*l2*SC*x(2)^2 - m2*(l2*x(4))^2);
            num2 = (m1+m2)*l2*g*s1 - m2*l2*g*s2*SC;
            den = l1*l2*((m1+m2) - m2*SC^2);
            y = (num1 + num2)/den;
        end
        
        function y = f2(obj, x)
            m1 = obj.m1; 
            m2 = obj.m2;
            l1 = obj.l1;
            l2 = obj.l2;
            g  = obj.g;
            
            s1 = sin(x(1)); s2 = sin(x(3));
            c1 = cos(x(1)); c2 = cos(x(3));
            SC = (s1*s2+c1*c2);
            
            num1 = (s1*c2-c1*s2)*(-(m1+m2)*(l1*x(2))^2 + m2*l1*l2*SC*x(4)^2);
            num2 = -(m1+m2)*l1*g*s1*SC + (m1+m2)*l1*g*s2;
            den = (m1+m2)*l2*g*s1 - m2*l2*g*s2*SC;
            y = (num1 + num2)/den;
        end
        
        function y = g11(obj, x)
            m1 = obj.m1; 
            m2 = obj.m2;
            l1 = obj.l1;

            s1 = sin(x(1)); s2 = sin(x(3));
            c1 = cos(x(1)); c2 = cos(x(3));
            SC = (s1*s2+c1*c2);
            
            den =  l1^2*((m1+m2) - m2*SC^2);
            y = 1/den;
        end
        
        function y = g12(obj, x) % g12=g22
            m1 = obj.m1; 
            m2 = obj.m2;
            l1 = obj.l1;
            l2 = obj.l2;

            s1 = sin(x(1)); s2 = sin(x(3));
            c1 = cos(x(1)); c2 = cos(x(3));
            SC = (s1*s2+c1*c2);
            
            num = -SC;
            den =  l1*l2*((m1+m2) - m2*SC^2);
            y = num/den;
        end
        
        function y = g22(obj, x)
            m1 = obj.m1; 
            m2 = obj.m2;
            l2 = obj.l2;

            s1 = sin(x(1)); s2 = sin(x(3));
            c1 = cos(x(1)); c2 = cos(x(3));
            SC = (s1*s2+c1*c2);

            num = m1+m2;
            den =  m2*l2^2*((m1+m2) - m2*SC^2);
            y = num/den;
        end
    end
end
%#ok<*PROPLC>