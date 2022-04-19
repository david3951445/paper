%Given a task space reference, let robot track it
%Given r(t) find q1(t)~q12(t)
% r(t) = [x(t); y(t)] : task space (planar) trajectory of robot
% CoM(t)              : Center of Mass trajectory

clc; clear; close all
addpath(genpath('../../../src'))
addpath(genpath('function'))

%% parameters
rb = Robot();
dt = 0.01; % Introduce time parameter into a path. If dt bigger, robot move slower (The points should more denser in this case)
splineDensity.r = 4; % relates to step length

%% Set task space (x-y plane) trajectory r(t)
% discrete waypoints for testing
r_dc = [
    0 0
    0 1
]/5;
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
qr = Ref2Config(rb, r, dt);
% qr = zeros(size(qr));
t = 0 : dt : dt*(length(qr)-1);
qr_L = cat(1, t, qr(1:2:11, :))';
qr_R = cat(1, t, qr(2:2:12, :))';

robot = rb.rbtree;
% showdetails(robot)
% show(robot, [qr_L(20, 2:7) qr_R(20, 2:7)]);

%% visualized robot joint traj
% n = length(qr);
% figure; hold on
% for i = 1 : 12
%     plot(t, qr(i, :)/pi*180, 'DisplayName', ['theta' num2str(i)])
% end
% legend

%% simulink
%% Contact and friction parameters
contact_stiffness = 40/.001;          % Approximated at weight (N) / desired displacement (m)
contact_damping = contact_stiffness/10; % Tuned based on contact stiffness value
mu_s = 0.9*1;     % Static friction coefficient: Around that of rubber-asphalt
mu_k = 0.8*1;     % Kinetic friction coefficient: Lower than the static coefficient
mu_vth = 0.1;   % Friction velocity threshold (m/s)

plane_x = 50;
plane_y = 3;
plane_z = 0.025; 

%% Robot mechanical Parameters (m)
% density = 1000;

leg_width = 0.08;
lower_leg_length = 0.38; 
upper_leg_length = 0.40;

foot_x = 0.08;
foot_y = 0.04;
foot_z = 0.005;

torso_width = 0.24; 
torso_length = 0.20; 
torso_height = 0.35; 

torso_offset_height = 0; 
torso_offset_length = 0; 

% world_damping = 0;      % Translational damping for 6-DOF joint [N/m]
% world_rot_damping = 0;  % Rotational damping for 6-DOF joint [N*m/(rad/s)]

%% Initial conditions
% Height of the 6-DOF joint between the ground and robot torso
init_height = rb.height_CoM0_walk + foot_z;
% Joint angles [hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll]
init_angs_R = zeros(6,1);
init_angs_L = zeros(6,1);

%% Robot joint parameters
joint_damping = 0;
motion_time_constant = 0.001;


