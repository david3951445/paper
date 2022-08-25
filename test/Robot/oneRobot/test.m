clc; clear; close all;
LEN = 100; n = 3;
f2    = 0.1*ones(1, LEN);
a = round(linspace(1,LEN,n));
b = [0.3 -0.2];
for i = 1 : n-1
    f2(:, a(i):a(i+1)) = b(i);
end
plot(linspace(0,1,LEN), f2)
