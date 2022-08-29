clc; clear; close all;
t = linspace(0, 10, 100);
x = square(t);
plot(t,x,'o');
hold on

fx = fit(t', x', 'SmoothingSpline');
len3 = 200;
t3 = linspace(0,10,len3);
x3 = feval(fx, t3)';
plot(t3, x3)

% fx = fit(t', t', 'SmoothingSpline');
% fy = fit(t', x', 'SmoothingSpline');
% t3 = linspace(0, 10, 200);
% r3 = [
%     feval(fx, t3)';
%     feval(fy, t3)';
% ];
% plot(r3(1,:),r3(2,:))