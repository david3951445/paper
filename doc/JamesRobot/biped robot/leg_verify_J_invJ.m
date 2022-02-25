clc;clear;
%%% 從物理意義去觀察J是否正確 %%% 
L4 = 0.11;
L5 = 0.11;
th = [0.01;0.21;0.2;0.2;0.1;0.11;0.1;0.1];
% verify the correctness of Jb 
Jb = leg_Jb(th,L4,L5);
% 觀察x_dot
th_dot = [1;0.1;2;0.2;0.5;0.5;0.3;0.3];
x_dot = Jb*th_dot;
% verify the correctness of invJb
invJb = leg_Jb_inv(th,L4,L5);
dth = invJb*x_dot;
