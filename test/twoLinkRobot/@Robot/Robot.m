classdef Robot
%robot model

properties (Constant, Access = private)
    m1 = 5, m2 = 3, l1 = 1, l2 = 2, g = 9.8;
end

properties
    % lpv system
    Al % A matrix of lpv system
    Bl % B matrix of lpv system
    ABl % AB solve together

    % linear systems
    A % A matrix of linear systems
    B % B matrix of linear systems
    AB % AB matrix of linear systems
    C % C matrix of linear systems
    K

    test = 0

end

properties (Access = private)
end

methods
    function o = Robot()
        % load old data
        RB_MAT = load('data/rb.mat');
        % o.A = RB_MAT.A;
        % o.B = RB_MAT.B;
        o.AB = RB_MAT.AB;
        o.K = RB_MAT.K;

        %% set lpv system          
        o.Al.val = {
            @(p)0               @(p)1               @(p)0               @(p)0
            @(p)o.setAl(p, 21)  @(p)o.setAl(p, 22)  @(p)o.setAl(p, 23)  @(p)o.setAl(p, 24)
            @(p)0               @(p)0               @(p)0               @(p)1
            @(p)o.setAl(p, 41)  @(p)o.setAl(p, 42)  @(p)o.setAl(p, 43)  @(p)o.setAl(p, 44)
        };
        o.Al.domain = 1*[-1 1; -1 1; -1 1; -1 1];
        o.Al.gridsize = 1*[10 10 10 10];
        o.Al.SV_TOLERANCE = 0.001;
        o.Al.num_p = length(o.Al.gridsize); % length of parameter vector of lpv system (p = [x1, x2, x3, x4])
        o.Al.dep = zeros([size(o.Al.val) o.Al.num_p]);
        o.Al.dep(2,1,:) = [1 0 1 0];
        o.Al.dep(2,2,:) = [1 1 1 0];
        o.Al.dep(2,3,:) = [1 0 1 0];
        o.Al.dep(2,4,:) = [1 0 1 1];
        o.Al.dep(4,1,:) = [1 0 1 0];
        o.Al.dep(4,2,:) = [1 1 1 0];
        o.Al.dep(4,3,:) = [1 0 1 0];
        o.Al.dep(4,4,:) = [1 0 1 1];

        o.Bl.val = {
            @(p)0               @(p)0
            @(p)o.setBl(p, 21)  @(p)o.setBl(p, 22)
            @(p)0               @(p)0
            @(p)o.setBl(p, 41)  @(p)o.setBl(p, 42)
        };
        o.Bl.domain = 2*[-1 1; -1 1];
        o.Bl.gridsize = 1*[10 10];
        o.Bl.SV_TOLERANCE = 0.001;
        o.Bl.num_p = length(o.Bl.gridsize); % length of parameter vector of lpv system (p = [x1, x2, x3, x4])
        o.Bl.dep = zeros([size(o.Bl.val) o.Bl.num_p]);
        o.Bl.dep(2,1,:) = [1 1];
        o.Bl.dep(2,2,:) = [1 1];
        o.Bl.dep(4,1,:) = [1 1];
        o.Bl.dep(4,2,:) = [1 1];

        o.ABl.val = {
            @(p)0               @(p)1               @(p)0               @(p)0               @(p)0               @(p)0
            @(p)o.setABl(p, 21) @(p)o.setABl(p, 22) @(p)o.setABl(p, 23) @(p)o.setABl(p, 24) @(p)o.setABl(p, 25) @(p)o.setABl(p, 26)
            @(p)0               @(p)0               @(p)0               @(p)1               @(p)0               @(p)0
            @(p)o.setABl(p, 41) @(p)o.setABl(p, 42) @(p)o.setABl(p, 43) @(p)o.setABl(p, 44) @(p)o.setABl(p, 45) @(p)o.setABl(p, 46)
        };
        o.ABl.domain = pi/2*[-1 1; -1 1; -1 1; -1 1;];
        o.ABl.gridsize = 20*[1 1 1 1];
        o.ABl.SV_TOLERANCE = 0.001;
        o.ABl.num_p = length(o.ABl.gridsize); % length of parameter vector of lpv system
        o.ABl.dep = zeros([size(o.ABl.val) o.ABl.num_p]);
        % A
        o.ABl.dep(2,1,:) = [1 0 1 0];
        o.ABl.dep(2,2,:) = [1 1 1 0];
        o.ABl.dep(2,3,:) = [1 0 1 0];
        o.ABl.dep(2,4,:) = [1 0 1 1];
        o.ABl.dep(4,1,:) = [1 0 1 0];
        o.ABl.dep(4,2,:) = [1 1 1 0];
        o.ABl.dep(4,3,:) = [1 0 1 0];
        o.ABl.dep(4,4,:) = [1 0 1 1];
        % o.ABl.dep(2,1,:) = [1 0 1 0 1 0];
        % o.ABl.dep(2,2,:) = [1 1 1 0 0 0];
        % o.ABl.dep(2,3,:) = [1 0 1 0 0 1];
        % o.ABl.dep(2,4,:) = [1 0 1 1 0 0];
        % o.ABl.dep(4,1,:) = [1 0 1 0 1 0];
        % o.ABl.dep(4,2,:) = [1 1 1 0 0 0];
        % o.ABl.dep(4,3,:) = [1 0 1 0 0 1];
        % o.ABl.dep(4,4,:) = [1 0 1 1 0 0];
        % B
        o.ABl.dep(2,5,:) = [1 0 1 0];
        o.ABl.dep(2,6,:) = [1 0 1 0];
        o.ABl.dep(4,5,:) = [1 0 1 0];
        o.ABl.dep(4,6,:) = [1 0 1 0];
        % o.ABl.dep(2,5,:) = [1 0 1 0 0 0];
        % o.ABl.dep(2,6,:) = [1 0 1 0 0 0];
        % o.ABl.dep(4,5,:) = [1 0 1 0 0 0];
        % o.ABl.dep(4,6,:) = [1 0 1 0 0 0];


        %% set linear system
        % o.A = [];
        % o.B = [];
        o.C = [1 0 0 0; 0 0 1 0];
    end
end

methods (Access = public)
    % [A, points] = setA(o)
    % [B, points] = setB(o)
    % [M, points] = TPmodelTransf(o)
    
    function y = f(o, x, u) % dx/dt
        y = [
            x(2)
            o.f1(x)+[o.g11(x) o.g12(x)]*u
            x(4)
            o.f2(x)+[o.g12(x) o.g22(x)]*u
        ];
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

    function save(o, fname, whichVar)
        switch whichVar
            case 'A'
                A = o.A;
            case 'B'
                B = o.B;
            case 'AB'
                AB = o.AB;
            case 'K'
                K = o.K;
            otherwise
                disp('No such property in Robot')
        end

        if isfile(fname)
            save(fname, whichVar, '-append');
        else
            save(fname, whichVar);
        end
    end
    
end

methods (Access = private)
    y = setAl(o, p, position)
    y = setBl(o, p, position)
    y = setABl(o, p, position)
%     function S = saveobj(o)
%         S = o;
%     end
end

end
%#ok<*PROPLC>
%#ok<*NASGU>