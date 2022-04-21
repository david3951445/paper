clc; clear; close all
n = 10;
x = rand(1, n);
y = rand(1, n);
f = figure;
plot(x, y, '.');
hold on
v = voronoi(x, y);
vx = v(2).XData;
vy = v(2).YData;
voronoi(x, y)
plot(vx, vy, 'o')
axis([0 1 0 1])