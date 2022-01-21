clc;clear;close all
A = [ 1 -3 -2;
     -1  1 -1;
      2  4  5];
% A = sym(A);
[V,J] = jordan(A)