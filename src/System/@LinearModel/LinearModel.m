classdef LinearModel
    %Linear system model
    % dx/dt = Ax + Bu + Ev
    % y = Cx
    
    properties
        A
        B
        C
        E

        Q1 % tracking
        Q2 % estimation
        R = []
        rho = 1
        
        DIM_X
        DIM_U
        DIM_Y
    end
    
    methods
        function obj = LinearModel(A, B, C)
            obj.A = A;
            obj.B = B;
            obj.C = C;

            [obj.DIM_X, obj.DIM_U] = size(B);
            [obj.DIM_Y, ~] = size(C);
            
            obj.E = eye(obj.DIM_X);
        end
    end
end

