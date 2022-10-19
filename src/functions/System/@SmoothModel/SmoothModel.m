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

        begin % begining index in the augment system
        fault % the fault signal of this model
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

        function xb = set_real_signal(obj, xb, i)
            %assign real fault signal
            % Since the real fault signal is not produced from smooth model, we need reassign it.
            j0 = obj.begin;
            range = 1 : obj.DIM;
            for j = 1 : obj.WINDOW-1
                j1 = j0 + j*obj.DIM;
                xb(j1+range, i+1) = xb(j1+range-obj.DIM, i); % Since Fa(i) = [fa(i), fa(i-1), fa(i-2), ...], so Fa(i+1) = ["new fa", fa(i), f(i-1)]
            end
            xb(j0+range, i+1) = obj.fault(:, i); % assign "new fa"
        end

        obj = getA_CT(obj); % construct continuos time smoothing model
        obj = getA_DT(obj); % construct discrete time smoothing mode
    end
end

