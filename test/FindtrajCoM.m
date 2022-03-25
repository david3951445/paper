%Given r(t) find CoM(t)
% r(t) = [x(t); y(t)]

clc; clear; close all
addpath(genpath('../../src'))

% discrete waypoints
r_dc = [
    0 0 1 1 1 2 3 3 2
    0 1 1 2 3 3 3 2 2
];
t = linspace(0, 1, length(r_dc));
xx = linspace(0, 1, length(r_dc)*11);
% using spline fit discrete waypoints to generate the r(t)
r = [
    spline(t, [0 r_dc(1, :) 0], xx)
    spline(t, [0 r_dc(2, :) 0], xx)
];

dr = my_diff(r); % numertial differentiation
r_l = dr; r_r = dr; % envelope of r(t)
L1 = 0.035; % Width between left and right foot
for i = 1 : length(r)
    N = (dr(1, i)^2 + dr(2, i)^2)^(1/2);
    r_l(:, i) = r(:, i) + [-dr(2, i); dr(1, i)]/N*L1;
    r_r(:, i) = r(:, i) + [dr(2, i); -dr(1, i)]/N*L1;
%     r_t(:, i) = r(:, i) + dr(:, i)/N*L1;
%     r_n(:, i) = r(:, i) + [-dr(2, i); dr(1, i)]/N*L1;
end

% Since the step length generate by spline seems to be appropriate, no need
% to set step length
% step = 0.2; % step length

plot(r(1, :), r(2, :), '-o')
hold on
plot(r_l(1, 1:2:length(r_l)), r_l(2, 1:2:length(r_l)), '-o')
plot(r_r(1, 2:2:length(r_l)), r_r(2, 2:2:length(r_l)), '-o')
% plot(r_t(1, :), r_t(2, :), 'b.') % tangent vector
% plot(r_n(1, :), r_n(2, :), 'b.') % normal vector
axis equal
title('foot trajectory')
legend('left foot', 'r(t)', 'right foot')

rmpath(genpath('../../src'))