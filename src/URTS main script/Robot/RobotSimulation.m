% Search task simuation
%
% Definition
% - transformation : homogeneous transformation
%  
clc; clear; close all

rb = Robot();
robot = rb.rbtree;
r = rb.tr.r{1}'; % reference
x = rb.tr.x(12 + (1:12), :);

% figure testing
% rr = r(1:1500:rb.tr.LEN, :)';
% tt = 1:1500:rb.tr.LEN;
% plot(tt, rr(4,:));
% hold on
% rrr = r(1:rb.tr.LEN, :)';
% plot(1:rb.tr.LEN, rrr(4,:));

factor = 50;
fps = 1/rb.tr.dt;
fps = fps/factor;

config = r(1:factor:length(x), :); % For simulatiom speed, compress the array by a factor
config = cat(2, config(:, 1:2:11), config(:, 2:2:12)); % To match the rigidbodytree config format, 1,2,...,12 -> 1,3,...11,2,4,...,12

%% State
transform = Config2LRSupportFeetTransform(robot, config, rb.INTERP_DENSITY/factor);

%% Reference
len = length(rb.r_lr);
r_lr = [rb.r_lr; zeros(1, len)];
% rotm = eul2rotm([-pi*.54 0 0]);
rotm = eul2rotm([0 0 0]);
for i = 1 : len   
    vec = rotm*(r_lr(:, i) - r_lr(:, 1));
    r_lr(:, i) = vec;
end

%% Plot
% Animation
% for i = 1 : length(config)
%     disp(i)
%     show(robot, config(i, :));
%     F{i} = getframe(gcf);
%     drawnow
% end

[fig, axisRange] = Show.Senario(); % Figure setting
Show.Area(axisRange)
Show.Map([0 9], [0 6]);

% Reference
plot3(r_lr(1, :), r_lr(2, :), r_lr(3, :),'-o','DisplayName', 'reference r(t)', LineWidth=1);

% State
p = poseplot("ENU", ScaleFactor=0.0002, MeshFileName='PlateSquareHoleSolid.stl', DisplayName='foothold paths');
for i = 1 : length(transform) % state
    p.Orientation = tform2rotm(transform{i});
    p.Position = tform2trvec(transform{i});
    % pos(:, i) = tform2trvec(transform{i});
    F{i} = getframe(gcf);
end
% plot3(pos(1, :), pos(2, :), pos(3, :), '-o', DisplayName='state')

MakeVideo(F, fps/5, 'data/footPrint.avi')

%% Function
