classdef LinearModel
    %Linear system model
    %
    % dx/dt = Ax + Bu + Ev
    % y = Cx
    %

    properties
        A
        B
        C
        E

        P1 % Lyapunov function of tracking error system
        P2 % Lyapunov function of estimation error system
        Q1 % Tracking weighting matrix
        Q2 % Estimation weighting matrix
        R = [] % Control effort weighting matrix
        rho = 1 % Prescribed attenuation value
        
        DIM_X % Dimension of state vector
        DIM_U % Dimension of control vector
        DIM_Y % Dimension of output vector
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

