%Given r(t) find CoM(t)
% r(t) = [x(t); y(t)] : planar trajectory of robot
% CoM(t)              : Center of Mass trajectory

clc; clear; close all
addpath(genpath('../../src'))

%% parameters
L = [.035 .0907 .0285 .11 .11 .0305]; % length pf links

%% findind left and right foot trajectory
% discrete waypoints for testing
% r_dc = [
%     0 0 1 1 1 2 3 3 2
%     0 1 1 2 3 3 3 2 2
% ];
r_dc = [
    0 1 1 0 1 2 3 4 5 5 4 4 2
    0 0 1 2 4 3 3 4 3 2 2 1 0
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
r_lr = dr;
L(1) = 0.035; % Width between left and right foot
for i = 1 : length(r)
    N = (dr(1, i)^2 + dr(2, i)^2)^(1/2);
    r_l(:, i) = r(:, i) + [-dr(2, i); dr(1, i)]/N*L(1);
    r_r(:, i) = r(:, i) + [dr(2, i); -dr(1, i)]/N*L(1);
    if mod(i, 2) == 1
        r_lr(:, i) = r_l(:, i);
    else
        r_lr(:, i) = r_r(:, i);
    end
%     r_t(:, i) = r(:, i) + dr(:, i)/N*L1;
%     r_n(:, i) = r(:, i) + [-dr(2, i); dr(1, i)]/N*L1;
end

% Since the step length generate by spline seems to be appropriate, no need
% to set step length
% step = 0.2; % step length
figure
hold on
plot(r_dc(1, :), r_dc(2, :), '-h', 'DisplayName', 'r_{dc}(t)')
plot(r(1, :), r(2, :), '-o', 'DisplayName', 'r(t)')
plot(r_lr(1, 1:2:length(r_l)), r_lr(2, 1:2:length(r_l)), '-o', 'DisplayName', 'left foot')
plot(r_lr(1, 2:2:length(r_l)), r_lr(2, 2:2:length(r_l)), '-o', 'DisplayName', 'right foot')
% plot(r_t(1, :), r_t(2, :), 'b.') % tangent vector
% plot(r_n(1, :), r_n(2, :), 'b.') % normal vector
axis equal
title('foot trajectory')

%% find ZMP
foot_size = [0.04 0.08];
t = linspace(0, 1, length(r_lr));
xx = linspace(0, 1, (length(r_lr)-1)*10+1);
zmp = [
    interp1(t, r_lr(1, :), xx)
    interp1(t, r_lr(2, :), xx)
];

plot(zmp(1, :), zmp(2, :), '-s', 'Displayname', 'ZMP trajectory')

%% find CoM
height = sum(L(2:6));
dt = 0.1;
t = 0 : dt : (length(zmp)-1)*dt;
CoM = ZMP2CoM(zmp, dt, height);

plot(CoM(1, :), CoM(2, :), 'Displayname', 'CoM trajectory')

legend

%% remove path
rmpath(genpath('../../src'))