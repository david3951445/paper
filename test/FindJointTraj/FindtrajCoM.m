%Given r(t) find CoM(t)
% r(t) = [x(t); y(t)] : planar trajectory of robot
% CoM(t)              : Center of Mass trajectory

clc; clear; close all
addpath(genpath('../../src'))

%% parameters
L                   = [.035 .0907 .0285 .11 .11 .0305]; % length pf links
splineDensity.r     = 12;
splineDensity.zmp   = 10;
height_CoM          = sum(L(2:6)) - .05; % remain .05 to let feet reach end point possibly
height_feet         = .05;
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
r_l = dr; r_r = dr; % envelope of r(t)
r_lr = dr; % traj of left-right feet
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
% r_r = r_lr(2:2:length(r_lr));

%% find left-right foot traj
% Since the step length generate by spline seems to be appropriate, no need
% to set step length
% step = 0.2; % step length
len2 = length(xx);
t = linspace(0, 1, len2);
len3 = (len2-1)*splineDensity.zmp+1;
xx = linspace(0, 1, len3);

r_l = [r_l; zeros(1, len2)];
r_l(3, 2:2:len2) = height_feet;
r_lh = [
    spline(t, [0 r_l(1, :) 0], xx);
    spline(t, [0 r_l(2, :) 0], xx);
    spline(t, [0 r_l(3, :) 0], xx);
];
r_r = [r_r; zeros(1, len2)];
r_r(3, 1:2:len2) = height_feet;
r_rh = [
    spline(t, [0 r_r(1, :) 0], xx);
    spline(t, [0 r_r(2, :) 0], xx);
    spline(t, [0 r_r(3, :) 0], xx);
];

%% find ZMP
zmp = [
    interp1(t, r_lr(1, :), xx)
    interp1(t, r_lr(2, :), xx)
];

%% find CoM
dt = 0.1;
t = 0 : dt : (length(zmp)-1)*dt;
CoM = ZMP2CoM(zmp, dt, height_CoM);
CoM(3, :) = height_CoM;

%% fig 1
part = 5;
len2 = round(len2/part);
len3 = round(len3/part);
figure
plot3(r_r(1, 1:len2), r_r(2, 1:len2), r_r(3, 1:len2), 'o', 'MarkerFaceColor', 'b', 'DisplayName', 'ground & highest')
hold on
plot3(r_l(1, 1:len2), r_l(2, 1:len2), r_l(3, 1:len2), 'o', 'MarkerFaceColor', 'b', 'DisplayName', 'ground & highest')
plot3(r_lh(1, 1:len3), r_lh(2, 1:len3), r_lh(3, 1:len3), '-o', 'DisplayName', 'left foot')
plot3(r_rh(1, 1:len3), r_rh(2, 1:len3), r_rh(3, 1:len3), '-o', 'DisplayName', 'right foot')
plot3(CoM(1, 1:len3), CoM(2, 1:len3), CoM(3, 1:len3), '-o', 'Displayname', 'CoM trajectory')
title(['1/' num2str(part) ' of left-right foot and CoM traj'])
legend
axis equal

%% fig 2
figure
hold on
plot(r_dc(1, :), r_dc(2, :), '-h', 'DisplayName', 'r_{dc}(t)')
plot(r(1, :), r(2, :), '-o', 'DisplayName', 'r(t)')
plot(r_lr(1, 1:2:length(r_l)), r_lr(2, 1:2:length(r_l)), '-o', 'DisplayName', 'left foot')
plot(r_lr(1, 2:2:length(r_l)), r_lr(2, 2:2:length(r_l)), '-o', 'DisplayName', 'right foot')
% plot(r_t(1, :), r_t(2, :), 'b.') % tangent vector
% plot(r_n(1, :), r_n(2, :), 'b.') % normal vector
plot(zmp(1, :), zmp(2, :), '-s', 'Displayname', 'ZMP trajectory')
plot(CoM(1, :), CoM(2, :), 'Displayname', 'CoM trajectory')
axis equal
title('foot trajectory')
xlabel('x'); ylabel('y')
legend

%% remove path
rmpath(genpath('../../src'))