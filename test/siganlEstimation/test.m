clc; clear; close all

r = 3; % r parameter for chaotic regime
t = 0 : 0.01 : 100;
x(1)= 1; % initial value
for i=1 :length(t)-1
   x(i+1) = r*x(i)*(1 - x(i));
end
plot(t, x)