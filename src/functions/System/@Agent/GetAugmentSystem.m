function [Ab, Bb, Cb] = GetAugmentSystem(ag)
    %construct augmented system, Fa + Fs
    % Ab = [A Ba*Ca 0; 0 Aa 0; 0 0 As]
    % Bb = [B; 0; 0]
    % Cb = [C 0 Bs*Cs]
    A = ag.sys.A; B = ag.sys.B; C = ag.sys.C;
    Aa = ag.sys_a.A; Ba = ag.sys_a.B; Ca = ag.sys_a.C;
    As = ag.sys_s.A; Bs = ag.sys_s.B; Cs = ag.sys_s.C;


    [DIM_X, DIM_U] = size(B);
    [DIM_Y, ~] = size(C);
    DIM_Xa = size(Aa, 1);
    DIM_Xs = size(As, 1);
    
    Ab3_2 = zeros(DIM_Xs, DIM_Xa);
    Ab = [
        A                        Ba*Ca  zeros(DIM_X, DIM_Xs)
        zeros(DIM_Xa, DIM_X)     Aa     zeros(DIM_Xa, DIM_Xs)
        zeros(DIM_Xs, DIM_X)     Ab3_2  As
    ];
    Bb = [B; zeros(DIM_Xa, DIM_U); zeros(DIM_Xs, DIM_U)];
    Cb = [C zeros(DIM_Y, DIM_Xa) Bs*Cs];
end
    
    