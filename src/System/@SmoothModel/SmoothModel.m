classdef SmoothModel
    %Smooth model
    % dx/dt = Ax = v
    % 

    properties
        METHOD % method for construct A
        WINDOW % window size
        A % system matrix
        B % input matrix
        C % mapping matrix
        Q1 % tracking weighting matrix of state in H infinity performance index
        Q2 % estimation weighting matrix of state in H infinity performance index
        dt % smapling time
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
        end
        
        obj = getA_CT(obj);
        obj = getA_DT(obj);
    end
end

