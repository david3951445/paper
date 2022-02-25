clc;clear;close all

x = 0:0.01:15;
y = sin(x);
maker_idx = 1:(length(x)-1)/2:length(x);
plot(x,y,'-s','MarkerIndices',maker_idx)
legend('y')