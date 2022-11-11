%A method to save and load a class in MATLAB
%
% Still testing

clc; clear;
addpath(genpath('..\function'))
c = Class1();
mfilename
a = 1;
% c.saveobj()
save([mfilename '.mat'], 'c');
% save mfilename c
rmpath(genpath('..\function'))