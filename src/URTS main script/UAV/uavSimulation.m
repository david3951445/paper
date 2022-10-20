clc; clear; close all

addpath(genpath('../../../src'))
uav = UAV_AGENTmodel();
DIM_F = uav.DIM_F;
dt = uav.tr.dt;
fps = 1/dt;
N = 100;
fps = fps/N;
r = uav.tr.r{1};
r = r(:, 1:N:length(r));
e = uav.tr.x(DIM_F+(1:DIM_F), :);
e = e(:, 1:N:length(e));
x = e + r;
% x(4:6, :) = x(4:6, :);
% drone_Animation(r(1,:),r(2,:),r(3,:),r(4,:),r(5,:),r(6,:))
drone_Animation(x(1,:),x(2,:),x(3,:),x(4,:),x(5,:),x(6,:), r, fps)