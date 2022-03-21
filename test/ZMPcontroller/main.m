clc; clear; close all

A = [0 1; 1 2]; B = [0; 1]; C = [1 0; 0 1]; D = [];
[DIM_X, DIM_U] = size(B);
dt = 0.001; t = 0 : dt : 5;

sys_CT = ss(A, B, C, D);
sys_DT = ss(A, B, C, D, dt);

K = lqr(sys_CT,eye(DIM_X),0.001*eye(DIM_U),[]);
u = zeros(DIM_U, length(t));
x0 = [-1; 1];
lsim(sys_CT, u, t, x0);