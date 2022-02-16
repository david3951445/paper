classdef LTI
    %LTI system 
    %
    % system form:
    %       dx/dt = Ax + Ev
    
    properties
        % Matrix
        A
        E

        % size
        SIZE_X

        % trajectory
        dt % time step
        t % time vector
        T % final time      
        x0 % initial state
        x % state
        v % disterbance

    end
    
    methods
        function obj = LTI(A, E, dt, T, x0, v)
            obj.A = A;
            obj.E = E;

            SIZE_X = size(A, 1);

            obj.dt  = dt; % time step
            obj.T   = T; % final time
            obj.t   = 0 : dt : T; % time vector
            obj.x0  = x0; % initial state      
            obj.v   = v; % disterbance
            
            % calculate x, u
            obj.x = zeros(SIZE_X, length(obj.t));
            obj.x(:, 1) = x0;
            for i = 1 : length(obj.t) - 1
                k1 = obj.RK4(obj.x(:, i),           i*dt);
                k2 = obj.RK4(obj.x(:, i)+k1*dt/2,   i*dt + dt/2);
                k3 = obj.RK4(obj.x(:, i)+k2*dt/2,   i*dt + dt/2);
                k4 = obj.RK4(obj.x(:, i)+k3*dt,     i*dt + dt);
        
                obj.x(:, i+1)  = obj.x(:, i)  + (k1 + 2*k2 + 2*k3 + k4)*dt/6;
            end
        end
    end

    methods (Access = private)
        function k = RK4(obj, x, t)
            k = obj.A*x + obj.E*obj.v(t);
        end
    end
end

