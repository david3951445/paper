clc;clear;close all
L3 = 0.0285;
L4 = 0.11;
L5 = 0.11;

th = [0 0 0 1.2];
A = leg_FK(th,L3,L4,L5);
th_IK = leg_IK(A,L3,L4,L5);
A_FK = leg_FK(th,L3,L4,L5);
