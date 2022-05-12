classdef SmoothModel
    %Smooth model
    % dx/dt = Ax + Ev
    % 

    properties
        METHOD % method for construct A
        WINDOW % window size  
        dt % smapling time

        A % system matrix
        B % input matrix
        C % mapping matrix
        E % disturbance matrix

        Q1 % tracking weighting matrix of state in H infinity performance index
        Q2 % estimation weighting matrix of state in H infinity performance index
        rho

        DIM % dimension of fault signal
        DIM_X % dimension of A
    end
    
    methods
        function obj = SmoothModel(WINDOW, DIM, dt, method)
            obj.WINDOW = WINDOW;
            obj.DIM = DIM;
            obj.DIM_X = DIM*WINDOW;
            obj.METHOD = method;

            % A
            if isempty(dt) % DT
                obj = obj.getA_DT();
            else          
                obj.dt = dt;
                obj = obj.getA_CT();
            end
            % C
            C = [1 zeros(1, WINDOW-1)];
            obj.C = kron(C, eye(obj.DIM));

            obj.E = eye(obj.DIM_X);
        end
        
        obj = getA_CT(obj);
        obj = getA_DT(obj);
    end
end

