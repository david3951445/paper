%Given r(t) find q1(t)~q12(t)
% r(t) = [x(t); y(t)] : task space (planar) trajectory of robot
% CoM(t)              : Center of Mass trajectory

clc; clear; close all
addpath(genpath('../../src'))

%% parameters
rb = Robot();
robot = rb.rbtree;
L                   = rb.L;
splineDensity.r     = 30;
splineDensity.zmp   = 20;
height_CoM          = rb.height_CoM;
height_feet         = rb.height_feet;
START_FOOT          = 1; % 1:left, 0:right

%% Set task space (x-y plane) trajectory r(t)
% discrete waypoints for testing
r_dc = [
    0 0 1
    0 1 1
];
% r_dc = [
%     0 0 1 1 1 2 3 3 2
%     0 1 1 2 3 3 3 2 2
% ];
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
    N = sqrt(dr(1, i)^2 + dr(2, i)^2);
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
[r_l, r_lh, frame_Lfoot] = FindLRfootTraj(r_l, splineDensity, ~START_FOOT+2, len2, height_feet);
[r_r, r_rh, frame_Rfoot] = FindLRfootTraj(r_r, splineDensity, START_FOOT, len2, height_feet);

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
% t = 0 : dt : (length(zmp)-1)*dt;
CoM = ZMP2CoM(zmp, dt, height_CoM);
CoM(3, :) = height_CoM;

%% find frames of CoM and sole of left-right foot
r1 = [
    spline(t, [0 r(1, :) 0], xx)
    spline(t, [0 r(2, :) 0], xx)
];
dr1 = my_diff(r1);
% dr_lh = my_diff(r_lh(1:2, :));
% dr_rh = my_diff(r_rh);
% N = sqrt(dr1(1, i)^2 + dr1(2, i)^2);
for i = 1 : len3
    frame_CoM(:, i) = [0 0 atan2(dr1(2, i), dr1(1, i))]';  % angle of z-axis between body and inertial
%     frame_Lfoot(:, i) = [0 0 atan2(dr_lh(2, i), dr_lh(1, i))]';
%     frame_Rfoot(:, i) = [0 0 atan2(dr_rh(2, i), dr_rh(1, i))]';
end

%% [x y z phi theta psi] -> config
ik = inverseKinematics('RigidBodyTree', robot);
weights = [1 1 1 1 1 1];
n = 100;
theta = zeros(6, n);

% CoM to inertial
frame0 = trvec2tform([0 0 0])*eul2tform([0 -rb.L(1) rb.L(2)]) % joint1 to CoM
frame1 = eye(4)/eul2tform(frame_CoM(:, 1)')/trvec2tform(CoM(:, 1)') % inertial to CoM
frame2 = trvec2tform(r_lh(:, 1)')*eul2tform(frame_Lfoot(:, 1)') % inertial to Lfoot
frame3 = trvec2tform([0 0 0])*eul2tform([-pi/2 -pi/2 0]); % Lfoot to Lfoot_endeffector

tform2eul(inv(frame2))
tform2trvec(frame2)
tform = frame0*frame1*frame2*frame3;

initialguess = [0 -0 -.2 .2 0 -.2];
[configSol,~] = ik('body7',tform,weights,initialguess);
% configSol/pi*180
% tform = getTransform(robot,configSol,'body7','base')
theta(:, 1) = configSol;
show(robot, configSol);

% for i = 2 : n
%     if mod(i, 10) == 0
%         disp(['i = ', num2str(i)])
%     end
%     tform = trvec2tform(pos(:, i)')*tform_r;
%     initialguess = configSol;
%     [configSol,~] = ik('body7',tform,weights,initialguess);
% %     configSol/pi*180
%     % tform = getTransform(robot,configSol,'body7','base')
%     theta(:, i) = configSol;
% end
% % show(robot, configSol);

% t = linspace(0,1,n);
% figure; hold on
% for i = 1 : 6
%     plot(t, theta(i, :), 'DisplayName', ['theta' num2str(i)])
% end
% legend

%% fig 1
% part = 1;
% len3_1 = round(len3/part);
% figure
% l = round(len2/part);
% plot3(r_r(1, 1:l), r_r(2, 1:l), r_r(3, 1:l), 'o', 'MarkerFaceColor', 'b', 'DisplayName', 'ground & highest')
% hold on
% plot3(r_l(1, 1:l), r_l(2, 1:l), r_l(3, 1:l), 'o', 'MarkerFaceColor', 'b', 'DisplayName', 'ground & highest')
% plot3(r_lh(1, 1:len3_1), r_lh(2, 1:len3_1), r_lh(3, 1:len3_1), '-o', 'DisplayName', 'left foot')
% plot3(r_rh(1, 1:len3_1), r_rh(2, 1:len3_1), r_rh(3, 1:len3_1), '-o', 'DisplayName', 'right foot')
% plot3(CoM(1, 1:len3_1), CoM(2, 1:len3_1), CoM(3, 1:len3_1), '-o', 'Displayname', 'CoM trajectory')
% title(['1/' num2str(part) ' of left-right foot and CoM traj'])
% legend
% axis equal

%% fig 2
% figure
% hold on
% plot(r_dc(1, :), r_dc(2, :), '-h', 'DisplayName', 'r_{dc}(t)')
% plot(r(1, :), r(2, :), '-o', 'DisplayName', 'r(t)')
% % plot(r_lr(1, 1:2:length(r_l)), r_lr(2, 1:2:length(r_l)), '-o', 'DisplayName', 'left foot')
% plot(r_l(1, :), r_l(2, :), 'o', 'DisplayName', 'left foot2')
% plot(r_lr(1, 2:2:len2), r_lr(2, 2:2:len2), '-o', 'DisplayName', 'right foot')
% plot(zmp(1, :), zmp(2, :), '-s', 'Displayname', 'ZMP trajectory')
% plot(CoM(1, :), CoM(2, :), 'Displayname', 'CoM trajectory')
% axis equal
% title('foot trajectory')
% xlabel('x'); ylabel('y')
% legend

%% fig 3, debug
% figure
% plot3(r_lh(1, 1:len3_1), r_lh(2, 1:len3_1), r_lh(3, 1:len3_1), '-o', 'DisplayName', 'left foot')
% hold on
% plot3(r_rh(1, 1:len3_1), r_rh(2, 1:len3_1), r_rh(3, 1:len3_1), '-o', 'DisplayName', 'right foot')
% grid on

% figure; hold on
% plot(r(1, :), r(2, :), '-o', 'DisplayName', 'r(t)')
% plot(r1(1, :), r1(2, :), '-h', 'DisplayName', 'r1(t)')
% legend


%% remove path
rmpath(genpath('../../src'))