function [Ab, Bb, Cb] = AugmentSystem1(A, B, C, Aa, Ba, Ca)
    %construct augment system, Fa
    % Ab = [A Ba*Ca; 0 Aa]
    % Bb = [B; 0]
    % Cb = [C 0]
    
    [DIM_X, DIM_U] = size(B);
    [DIM_Y, ~] = size(C);
    DIM_Xa = size(Aa, 1);
    
    Ab = [
        A                        Ba*Ca
        zeros(DIM_Xa, DIM_X)     Aa    
    ];
    Bb = [B; zeros(DIM_Xa, DIM_U)];
    Cb = [C zeros(DIM_Y, DIM_Xa)];
    end
    
    