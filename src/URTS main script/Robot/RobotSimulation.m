clc; clear; close all
rb = load("data/Robot.mat");

robot = rb.rbtree;
r = rb.tr.r{1}'; % reference
x = rb.tr.x(12 + (1:12), :);

% figure
% rr = r(1:1500:rb.tr.LEN, :)';
% tt = 1:1500:rb.tr.LEN;
% plot(tt, rr(4,:));
% hold on
% rrr = r(1:rb.tr.LEN, :)';
% plot(1:rb.tr.LEN, rrr(4,:));

factor = 50;
fps = 1/rb.tr.dt;
fps = fps/factor;

% r = rb.tr.r{1}(:, 1:rb.tr.LEN)'; % Trim to the length matching the simulation time
config = r(1:factor:length(x), :); % For simulatiom speed, compress the array by a factor
config = cat(2, config(:, 1:2:11), config(:, 2:2:12)); % To match the rigidbodytree config format, 1,2,...,12 -> 1,3,...11,2,4,...,12

% Animation
% for i = 1 : length(config)
%     disp(i)
%     show(robot, config(i, :));
%     F{i} = getframe(gcf);
%     drawnow
% end

% showdetails(robot);
numberOfFrameInHalf = rb.INTERP_DENSITY/factor; % The number of "animation frame" in a half walking cycle
supportFeetChanged = 1 + numberOfFrameInHalf/2; 
isLeftFoot = false;

index = 1;
supportFeetTransform = trvec2tform([rb.r_lr(:, 1)' 0])*eul2tform([pi*.225 0 0]);
transform{index} = supportFeetTransform;
transformRef{index} = trvec2tform([rb.r_lr(:, 1)' 0]);
for i = 2 : size(config, 1)
    if i == supportFeetChanged + 1
        isLeftFoot = ~isLeftFoot;
        % leftRobot = subtree(robot, 'body_f1');
        % rightRobot = subtree(robot, 'body_f2');
        % frame1 = getTransform(leftRobot, config(i, :), 'body11'); % joint1 to joint11
        % frame2 = getTransform(rightRobot, config(i, :), 'body12');
        if isLeftFoot
            tf = getTransform(robot, config(i, :), 'body12', 'body11'); % Left foot is the supporting foot
            % tf = eye(4)/frame1*frame2;
        else
            tf = getTransform(robot, config(i, :), 'body11', 'body12'); % Right foot is the supporting foot
            % tf = eye(4)/frame2*frame1;
        end

        supportFeetChanged = supportFeetChanged + numberOfFrameInHalf;
        supportFeetTransform = supportFeetTransform*eul2tform([0 -pi/2 -pi/2])*tf/eul2tform([0 -pi/2 -pi/2]);
        index = index + 1;
        
        transform{index} = supportFeetTransform;
        transformRef{index} = trvec2tform([rb.r_lr(:, index)' 0]);
        % transform{index} = transformRef{index-1}*eul2tform([0 -pi/2 -pi/2])*tf/eul2tform([0 -pi/2 -pi/2]);

        % show(robot, config(i, :));
        % F{index-1} = getframe(gcf);
        % drawnow
    end

    % if isLeftFoot
    %     tf = getTransform(robot, config(i, :), 'body_f1', 'body_f2'); % Initially the left foot is the supporting foot
    % else
    %     tf = supportFeetTransform;
    % end
    
    % transform{i} = tf;
end
show(robot, config(16, :));
getTransform(robot, config(16, :), 'body11', 'body12');

figure
% plotTransforms(se3(transform{1}))
% len = 30;
len = length(transform);
pos = zeros(3, len);
for i = 1 : len%length(transform)
    pos(:, i) = tform2trvec(transform{i});
    posRef(:, i) = tform2trvec(transformRef{i});
end
plot3(pos(1, :), pos(2, :), pos(3, :), '-o', 'DisplayName', 'state', 'LineWidth', 1)
hold on
plot3(posRef(1, :), posRef(2, :), posRef(3, :), '-o', 'DisplayName', 'reference', 'LineWidth', 1)
axis equal;
grid on;
% view(150,20);
view(0,90);
% xlim([-.5 10.5]); ylim([-.5 10.5]); zlim([-1.5 1.5]);
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');
legend('Interpreter','latex','Location','best');
% plotTransforms(se3(transform{100}))

% disp('animationing ...')
% F = cell(1, length(transform));
% for i = 1 : length(transform)
%     plotTransforms(se3(transform{i}))
%     % F{i} = getframe(gcf);
% end

% disp('making .avi ...')
% %% save frame to .avi
% writerObj = VideoWriter('data/footPrint.avi');
% writerObj.FrameRate = fps/5;
% open(writerObj);
%  for i = 1 : length(F)
%     writeVideo(writerObj, F{i});
%  end
%  close(writerObj);