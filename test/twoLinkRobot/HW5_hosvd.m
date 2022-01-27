clc; clear; close all
addpath(genpath('..\..\src'))

rb = Robot();
if EXE.SET_A
    rb.A = TPmodel(rb.Al);
    save('rb.mat', 'rb')
else
    load('rb.mat', 'rb')
end 

if EXE.SET_B
    rb.B = TPmodel(rb.Bl);
    save('rb.mat', 'rb')
else
    load('rb.mat', 'rb')
end

sum = 0;
for i = 1 : rb.A.len
    sum = sum + rb.A.mf([0, 0, 0, 0], i);
end
disp(['sum of mbfun of A: ' num2str(sum)])
sum = 0;
for i = 1 : rb.B.len
    sum = sum + rb.B.mf([0, 0, 0, 0], i);
end
disp(['sum of mbfun of B: ' num2str(sum)])