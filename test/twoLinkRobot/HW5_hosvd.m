clc; clear; close all
addpath(genpath('..\..\src'))

rb = Robot();
if EXE.SET_A
    rb.A = TPmodelTransf(rb.Al);
    save('rb.mat', 'rb')
else
    load('rb.mat', 'rb')
end 

if EXE.SET_B
    rb.B = TPmodelTransf(rb.Bl);
    save('rb.mat', 'rb')
else
    load('rb.mat', 'rb')
end

rb.B.mf = @rb.mf_B;
B = rb.B.val;
index = Combvec(rb.B.sizeO);
sum = 0;
for i = 1 : length(B)
    sum = sum + rb.B.mf([0, 0, 0, 0], index(:, i));
end
sum