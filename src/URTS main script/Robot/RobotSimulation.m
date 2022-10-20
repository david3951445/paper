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

%% Animation
% for i = 1 : length(config)
%     disp(i)
%     show(robot, config(i, :));
%     F{i} = getframe(gcf);
%     drawnow
% end

%% State
isLeftFoot = true;
index = 1;
supportFeetTransform = eul2tform([0 0 0]);
% supportFeetTransform = eul2tform([-pi/2 0 0]);
numberOfFrameInHalf = rb.INTERP_DENSITY/factor; % The number of "animation frame" in a half walking cycle
supportFeetChanged = 1 + numberOfFrameInHalf/2; 
transform{index} = supportFeetTransform;
for i = 2 : size(config, 1)
    if i == supportFeetChanged + 1      
        if isLeftFoot
            tf = getTransform(robot, config(i, :), 'body12', 'body11'); % Left foot is the supporting foot
        else
            tf = getTransform(robot, config(i, :), 'body11', 'body12'); % Right foot is the supporting foot
        end

        % Update
        supportFeetChanged = supportFeetChanged + numberOfFrameInHalf;
        supportFeetTransform = supportFeetTransform*eul2tform([0 -pi/2 -pi/2])*tf/eul2tform([0 -pi/2 -pi/2]);
        index = index + 1;
        isLeftFoot = ~isLeftFoot;

        transform{index} = supportFeetTransform*eul2tform([0 -pi/2 -pi/2]);

        % show(robot, config(i, :));
        % F{index-1} = getframe(gcf);
    end
end

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
[fig, axisRange] = Show.Senario(); % Figure setting
Show.Area(axisRange)
Show.Map()

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