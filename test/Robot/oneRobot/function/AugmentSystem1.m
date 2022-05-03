function [Ab, Bb, Cb] = AugmentSystem1(sys, sys_a)
%construct augment system, Fa + Fs
% Ab = [A B*C; 0 Aa]
% Bb = [B; 0]
% Cb = [C 0]

[DIM_X, DIM_U] = size(sys.B);
[DIM_Y, ~] = size(sys.C);

Ab = [
    sys.A                         sys_a.B*sys_a.C
    zeros(sys_a.DIM_X, DIM_X)     sys_a.A
];
Bb = [sys.B; zeros(sys_a.DIM_X, DIM_U)];
Cb = [sys.C zeros(DIM_Y, sys_a.DIM_X)];
end

