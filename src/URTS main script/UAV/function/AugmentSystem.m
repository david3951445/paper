function [Ab, Bb, Cb] = AugmentSystem(A, B, C, Aa, Ba, Ca, As, Bs, Cs)
%construct augment system, Fa + Fs
% Ab = [A Ba*Ca 0; 0 Aa 0; 0 0 As]
% Bb = [B; 0; 0]
% Cb = [C 0 Bs*Cs]

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

