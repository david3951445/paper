classdef UAV_AGENTmodel < Agent 
% agent model
  
    properties (Constant)
        % system parameters
        % l = 0.2, b = 2, d = 5, 
        m = 2, G = 9.81
        Jx = 1.25, Jy = 1.25, Jz = 2.2
        Kx = 0.01, Ky = 0.01, Kz = 0.01
        Kph = 0.012, Kth = 0.012, Kps = 0.012
        dt = .001

        %% flow control of code
    end

    properties
        Dt % torque drag coefficient matrix
        Df % force drag coefficient matrix

        qr
    end
  
    methods
        function uav = UAV_AGENTmodel()
            uav@Agent();
            uav.PATH  = ['data/' mfilename]; % path of saved data
            uav = uav.Load(); % load old data
            uav.DIM_F = 6;
            
            uav.Dt = diag([uav.Kph, uav.Kth, uav.Kps]);
            uav.Df = diag([uav.Kx, uav.Ky, uav.Kz]);                 
        end

        function y = M(uav, x)
            y = [
                uav.m*eye(3) zeros(3)
                zeros(3) diag([uav.Jx, uav.Jy, uav.Jz])
            ];
        end

        function y = H(uav, x, dx)
            CG = [[0;0;uav.G]; cross(dx(4:6), diag([uav.Jx, uav.Jy, uav.Jz])*dx(4:6))];
            F = [uav.Df zeros(3);zeros(3) uav.Dt]*dx;

            y = CG + F;
        end
        
        function y = R_inv(uav, x)
            y = rotx(-x(1))*roty(-x(2))*rotz(-x(3));
        end

        function y = R(uav, x)
            y = rotz(x(3))*roty(x(2))*rotx(x(1));
        end

        function y = u_fb(uav, xh)
            % range = 1 : uav.DIM_F*3; 
            % y = uav.K(:, range)*xh(range); % without unknown sigal info
            y = uav.K*xh; 
        end

        function [phi, theta, F] = pos_controller(uav, xh, r4, dt)  % c = [fx; fy; fz] = R(Theta)*[0; 0; F]
            % In practice, r([1 2 3], i) get from path planning algorithm.
            % UAV no spin, r(6, i) is zeros. r([4 5], i) is obtain from r([1 2 3 6], i)

            ddr = r4*[1 -2 1]'/dt^2;
            u = uav.u_fb(xh);
            c = uav.m*eye(3)*ddr + [0; 0; -uav.m*uav.G] + uav.Df*xh(2*uav.DIM_F + (1:3));%+u(1:3);
            % c = u(1:3);

            F = sqrt(c(1)^2 + c(2)^2 + c(3)^2);
            theta = atan(c(1)/c(3));
            phi = atan(-c(2)*cos(theta)/c(3));
        end

        function y = f_aug(uav, t, xb)
            DIM_X3 = uav.sys_aug.DIM_X;
            x = xb(1 : DIM_X3);
            xh = xb(DIM_X3 + (1:DIM_X3));
            u = uav.u_fb(xh);
            y = [
                uav.sys_aug.A*x + uav.sys_aug.B*u
                uav.sys_aug.A*xh + uav.sys_aug.B*u - uav.KL*uav.sys_aug.C*(x-xh)
            ];
        end
    end

    methods (Access = private)
    end
end
%#ok<*PROPLC>
%#ok<*NASGU>