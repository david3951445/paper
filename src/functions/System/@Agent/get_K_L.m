function ag = get_K_L(ag, sys1, sys_aug1)
I = eye(ag.DIM_F);

if ag.EXE_LMI
    disp('solving LMI ...')
    [ag.K, ag.KL, P1, P2] = solveLMI10(sys_aug1.A, sys_aug1.B, sys_aug1.C, sys_aug1.E, sys_aug1.Q1, sys_aug1.Q2, sys_aug1.R, sys_aug1.rho);
    ag.sys_aug.P1 = kron(P1, I);
    ag.sys_aug.P2 = kron(P2, I);
%     norm(ag.KL)

    ag.Save('K') 
    ag.Save('KL')
    ag.Save('sys_aug')
end

%% Fine tune of gain
% feedback control law u_fb = K*x_b, where K = [Kp Ki Kd Ka Ks]

gain = [-1 zeros(1, ag.sys_a.WINDOW-1)]; % since the optimal actuator fault gain Ka is -I so that it can offset the estimation of actuator fault f_a.
ag.K(:, sys1.DIM_X + (1:ag.sys_a.WINDOW)) = gain;
% ag.KL(4:6, :) = [1000 0 0; 0 100 0; 0 0 10];
% ag.KL(7:9,:) = [1000 0 0; 0 100 0; 0 0 10];
% ag.K(:, sys1.DIM_X + sys_a.WINDOW + (1:sys_s.WINDOW)) = zeros(1, sys_s.WINDOW);
% I0 = diag(1.1.^(0:ag.DIM_F-1));
% ...

%% construt origin gain
ag.K = kron(ag.K, I);
ag.KL = kron(ag.KL, I);
end