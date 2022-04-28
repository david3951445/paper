function sys_aug = AugmentSystem(sys, sys_a, sys_s)
%construct augment system
% Ab = [A B*C 0; 0 Aa 0; 0 0 As]
% Bb = [B; 0; 0]
% Cb = [C 0 Cs]

[DIM_X, DIM_U] = size(sys.B);
[DIM_Y, ~] = size(sys.C);
Ab3_2 = zeros(sys_s.DIM_X, sys_a.DIM_X);

Ab = [
    sys.A                         sys_a.B*sys_a.C zeros(DIM_X, sys_s.DIM_X)
    zeros(sys_a.DIM_X, DIM_X)     sys_a.A       zeros(sys_a.DIM_X, sys_s.DIM_X)
    zeros(sys_s.DIM_X, DIM_X)     Ab3_2         sys_s.A
];
Bb = [sys.B; zeros(sys_a.DIM_X, DIM_U); zeros(sys_s.DIM_X, DIM_U)];
Cb = [sys.C zeros(DIM_Y, sys_a.DIM_X) sys_s.B*sys_s.C];

sys_aug.A = Ab;
sys_aug.B = Bb;
sys_aug.C = Cb;
[sys_aug.DIM_X, sys_aug.DIM_U] = size(Bb);
[sys_aug.DIM_Y, ~] = size(Cb);
end

