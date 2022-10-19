function rb = Ref2Config(rb, r)
%Given r(t) find qr1(t)~qr12(t)
% r(t) = [x(t); y(t)] : Task space (planar) trajectory of robot
% qr1(t)~qr12(t)      : Joint space trajectory

%% parameters
robot               = rb.rbtree;
L                   = rb.L;
dt                  = rb.tr.dt;
splineDensity       = rb.INTERP_DENSITY;
height_feet         = rb.height_feet;
START_FOOT          = 1; % 1:left, 0:right
len2                = length(r);
dr                  = my_diff(r); % numertial differentiation

%% find left-right foot traj
% x, y of left-right foot traj
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

% z, phi, theta, psi of left-right foot traj
[r_l, r_lh] = FindLRfootTraj(r_l, splineDensity, ~START_FOOT+2, len2, height_feet);
[r_r, r_rh] = FindLRfootTraj(r_r, splineDensity, START_FOOT, len2, height_feet);

%% find ZMP traj
t = linspace(0, 1, len2);
len3 = (len2-1)*splineDensity+1;
xx = linspace(0, 1, len3);
zmp = [
    interp1(t, r_lr(1, :), xx)
    interp1(t, r_lr(2, :), xx)
];

%% find CoM traj
% x, y, z of CoM
% t = 0 : dt : (length(zmp)-1)*dt;
if rb.EXE_Z2C
    CoM = ZMP2CoM(zmp, dt, rb.height_CoM); % x, y
    CoM(3, :) = rb.height_CoM; % z

    % phi, theta, psi of CoM
    r1 = [
        spline(t, [0 r(1, :) 0], xx)
        spline(t, [0 r(2, :) 0], xx)
    ];
    dr1 = my_diff(r1);
    frame_CoM = zeros(3, len3);
    for i = 1 : len3
        frame_CoM(:, i) = [0 0 atan2(dr1(2, i), dr1(1, i))]';  % angle of z-axis between body and inertial
    end
    rb.CoM = cat(1, CoM, frame_CoM);
    rb.Save('CoM');
end
CoM0 = rb.CoM;
CoM0(3, :) = rb.height_CoM0_walk;

%% Inverse Kinemic, [x y z phi theta psi] -> config
if rb.EXE_IK
    n = rb.tr.LEN;
    qr = zeros(12, n);
    newSubtree = subtree(robot, 'body_f1');
    qr(1:2:11, :) = IK_leg(newSubtree, CoM0(:, 1:n), r_lh(:, 1:n));
    % show(newSubtree, qr(1:2:11, 1)');
    newSubtree = subtree(robot, 'body_f2');
    qr(2:2:12, :) = IK_leg(newSubtree, CoM0(:, 1:n), r_rh(:, 1:n));
    % show(newSubtree, qr(2:2:12, 1)');
    rb.qr = qr;
    rb.Save('qr');
end

%% Save data
rb.r_lr = r_lr;
rb.Save('r_lr');
rb.zmp = zmp;

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
% % plot(r_dc(1, :), r_dc(2, :), '-h', 'DisplayName', 'r_{dc}(t)')
% plot(r(1, :), r(2, :), '-o', 'DisplayName', 'r(t)')
% plot(r_lr(1, 1:2:length(r_l)), r_lr(2, 1:2:length(r_l)), '-o', 'DisplayName', 'left foot')
% % plot(r_l(1, :), r_l(2, :), 'o', 'DisplayName', 'left foot2')
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
end