clc; clear; close all
addpath(genpath('..\..\src'))

rb = Robot();
if EXE.SET_A
    [rb.A, rb.mf_A_points] = rb.setA();
    save('rb.mat', 'rb')
else
    load('rb.mat', 'rb')
end 

if EXE.SET_B
    rb.B = rb.setB();
    save('rb.mat', 'rb')
else
    load('rb.mat', 'rb')
end

A = rb.A;
sum = 0;
for i1 = 1:size(A, 1)
    for i2 = 1:size(A, 2)
        for i3 = 1:size(A, 3)
            for i4 = 1 : size(A, 4)
               sum = sum + rb.mf_A([0, 0, 0, 0], [i1, i2, i3, i4]);
            end
        end        
    end
end
sum