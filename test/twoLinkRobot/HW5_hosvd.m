clc; clear; close all
addpath(genpath('..\..\src'))

rb = Robot();
if EXE.SET_A
    rb.A = rb.setA();
    save('rb.mat', 'rb')
else
    load('rb.mat', 'rb')
end 

rb.B = rb.setB();