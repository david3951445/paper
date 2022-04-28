%DT system

clc; clear; close all
addpath(genpath('../../src'))

dt = .001; t = 0 : dt : 10;
%% sys
A = [0 1 0; 0 0 1; 0 0 0]; B = [0; 0; 1]; C = eye(3); D = [0; 0; 0];
DIM_F = 2;
A = kron(A, eye(DIM_F));
B = kron(B, eye(DIM_F));
C = kron(C, eye(DIM_F));
D = kron(D, eye(DIM_F));
[DIM_X, DIM_U] = size(B);
[DIM_Y, ~] = size(C);

sys = ss(A,B,C,D);
sys_DT = c2d(sys, dt);
%% smooth model
WINDOW = 3; DIM = 1*DIM_F;
sys_a = SmoothModel(WINDOW, DIM, []);
WINDOW = 2; DIM = DIM_Y;
sys_s = SmoothModel(WINDOW, DIM, []);
%% augment sys
[Ab, Bb, Cb] = AugmentSystem(sys, sys_a, sys_s);
