clc; clear; close all
addpath(genpath('..\..\src'))

rb = Robot();
if EXE.SET_A
%     [rb.A, rb.mf_A_points] = rb.setA();
    rb.A = TPmodelTransf(rb.Al, 'A');
    save('rb.mat', 'rb')
else
    load('rb.mat', 'rb')
end 

if EXE.SET_B
    rb.B = TPmodelTransf(rb.Bl, 'B');
    save('rb.mat', 'rb')
else
    load('rb.mat', 'rb')
end

B = rb.B.val;
index = Combvec(rb.B.sizeO);
sum = 0;
for i = 1 : length(B)
    sum = sum + rb.mf_B([0, 0, 0, 0], index(:, i));
end
sum