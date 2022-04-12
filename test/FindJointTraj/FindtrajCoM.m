%Given r(t) find CoM(t)
% r(t) = [x(t); y(t)] : planar trajectory of robot
% CoM(t)              : Center of Mass trajectory

clc; clear; close all
addpath(genpath('../../src'))

%% parameters
L                   = [.035 .0907 .0285 .11 .11 .0305]; % length pf links
splineDensity.r     = 30;
splineDensity.zmp   = 10;
height_CoM          = sum(L(2:6)) - .05; % remain .05 to let feet reach end point possibly
height_feet         = .05;
START_FOOT          = 1; % 1:left, 0:right

%% Set task space (x-y plane) trajectory r(t)
% discrete waypoints for testing
r_dc = [
    0 0 1 1 1 2 3 3 2
    0 1 1 2 3 3 3 2 2
];
% r_dc = [
%     0 1 1 0 1 2 3 4 5 5 4 4 3 2
%     0 0 1 2 4 3 3 4 3 2 2 1 .5 0
% ];

len1 = length(r_dc);
t = linspace(0, 1, len1);
len2 = len1*splineDensity.r;
xx = linspace(0, 1, len2);
% using spline fit discrete waypoints to generate the r(t)
r = [
    spline(t, [0 r_dc(1, :) 0], xx)
    spline(t, [0 r_dc(2, :) 0], xx)
];
dr = my_diff(r); % numertial differentiation

%% find left-right foot traj
r_l = zeros(size(dr)); r_r = zeros(size(dr)); % envelope of r(t)
r_lr = zeros(size(dr)); % traj of left-right feet
for i = 1 : length(r)
    N = (dr(1, i)^2 + dr(2, i)^2)^(1/2);
    r_l(:, i) = r(:, i) + [-dr(2, i); dr(1, i)]/N*L(1);
    r_r(:, i) = r(:, i) + [dr(2, i); -dr(1, i)]/N*L(1);
    if mod(i, 2) == START_FOOT
        r_lr(:, i) = r_l(:, i);
    else
        r_lr(:, i) = r_r(:, i);
    end
%     r_t(:, i) = r(:, i) + dr(:, i)/N*L1;
%     r_n(:, i) = r(:, i) + [-dr(2, i); dr(1, i)]/N*L1;
end

% Since the step length generate by spline seems to be appropriate, no need
% to set step length (about 0.1 for this robot)
[r_l, r_lh] = FindLRfootTraj(r_l, splineDensity, ~START_FOOT+2, len2, height_feet);
[r_r, r_rh] = FindLRfootTraj(r_r, splineDensity, START_FOOT, len2, height_feet);

%% find ZMP
t = linspace(0, 1, len2);
len3 = (len2-1)*splineDensity.zmp+1;
xx = linspace(0, 1, len3);
zmp = [
    interp1(t, r_lr(1, :), xx)
    interp1(t, r_lr(2, :), xx)
];

%% find CoM
dt = 0.1; % Introduce time parameter into a path. If dt bigger, robot move slower (The points should more denser in this case)
t = 0 : dt : (length(zmp)-1)*dt;
CoM = ZMP2CoM(zmp, dt, height_CoM);
CoM(3, :) = height_CoM;

%% fig 1
part = 1;
len3_1 = round(len3/part);
figure
l = round(len2/part);
plot3(r_r(1, 1:l), r_r(2, 1:l), r_r(3, 1:l), 'o', 'MarkerFaceColor', 'b', 'DisplayName', 'ground & highest')
hold on
plot3(r_l(1, 1:l), r_l(2, 1:l), r_l(3, 1:l), 'o', 'MarkerFaceColor', 'b', 'DisplayName', 'ground & highest')
plot3(r_lh(1, 1:len3_1), r_lh(2, 1:len3_1), r_lh(3, 1:len3_1), '-o', 'DisplayName', 'left foot')
plot3(r_rh(1, 1:len3_1), r_rh(2, 1:len3_1), r_rh(3, 1:len3_1), '-o', 'DisplayName', 'right foot')
plot3(CoM(1, 1:len3_1), CoM(2, 1:len3_1), CoM(3, 1:len3_1), '-o', 'Displayname', 'CoM trajectory')
title(['1/' num2str(part) ' of left-right foot and CoM traj'])
legend
axis equal

%% fig 2
figure
hold on
plot(r_dc(1, :), r_dc(2, :), '-h', 'DisplayName', 'r_{dc}(t)')
plot(r(1, :), r(2, :), '-o', 'DisplayName', 'r(t)')
% plot(r_lr(1, 1:2:length(r_l)), r_lr(2, 1:2:length(r_l)), '-o', 'DisplayName', 'left foot')
plot(r_l(1, :), r_l(2, :), 'o', 'DisplayName', 'left foot2')
plot(r_lr(1, 2:2:len2), r_lr(2, 2:2:len2), '-o', 'DisplayName', 'right foot')
% plot(r_t(1, :), r_t(2, :), 'b.') % tangent vector
% plot(r_n(1, :), r_n(2, :), 'b.') % normal vector
plot(zmp(1, :), zmp(2, :), '-s', 'Displayname', 'ZMP trajectory')
plot(CoM(1, :), CoM(2, :), 'Displayname', 'CoM trajectory')
axis equal
title('foot trajectory')
xlabel('x'); ylabel('y')
legend

%% fig 3, debug
figure
plot3(r_lh(1, 1:len3_1), r_lh(2, 1:len3_1), r_lh(3, 1:len3_1), '-o', 'DisplayName', 'left foot')
grid on

%% remove path
rmpath(genpath('../../src'))