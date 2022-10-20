% Simulation of team1 for all search task
clc; clear; close all
addpath(genpath('../../../src'))

%% Paths for robots
startGoal = [0.75 0 0; 0.75 6.5 0];
Na = 5; % number of agents in a team
for i = 1 : Na-1
    disp(['Path planning, ' num2str(i) '-th robot'])
    rb{i} = Robot();
    pp{i} = PathPlanning(startGoal(1, :), startGoal(2, :)); % Find task space ref in a map using RRT
    rb{i}.EXE_Z2C = 0;
    rb{i}.EXE_IK = 0;
    rb{i} = rb{i}.Ref2Config(pp{i}.r);

    startGoal(:, 1) = startGoal(:, 1) + 1.5;
end

%% Coordinate conversion
[pp, rb, map] = CoordConversion(pp, rb);

%% Plot the occupancy map and path planning result
% fig = figure;
% hold on
% axis equal
% xlabel('X (m)'); ylabel('Y (m)')
% legend('Interpreter','latex','Location','best')
% show(pp{1}.map);

[fig, axisRange] = Show.Senario();
Show.Area([map.x; map.y; 0 4])
Show.Map(map.x, map.y);

PP = pp{1}; RB = rb{1};
% plotObj{1} = plot(PP.sigma(1, :), PP.sigma(2, :), 'o', 'DisplayName', '$\sigma(t)$'); % path (from path planning)
% plotObj{2} = plot(PP.r(1, :), PP.r(2, :), '.', DisplayName='$\sigma''(t)$'); % smoothed path
% len1 = length(RB.r_lr);
% plotObjLeftFoot = plot(RB.r_lr(1, 1:2:len1), RB.r_lr(2, 1:2:len1), 'square', 'DisplayName', 'left foot');
% plotObjRightFoot = plot(RB.r_lr(1, 2:2:len1), RB.r_lr(2, 2:2:len1), 'o', 'DisplayName', 'right foot');
% for i = 2 : length(rb)
    % PP = pp{i}; RB = rb{i};  
    % plot(p.tree(:,1), p.tree(:,2),'.-', 'DisplayName','RRT tree expansion'); % tree expansion
    % AddXYData(plotObj{1}, PP.sigma(1, :), PP.sigma(2, :))
    % AddXYData(plotObj{2}, PP.r(1, :), PP.r(2, :))

    % len = length(PP.sigma);
    % plot(PP.sigma(1, 1), PP.sigma(2, 1), 'square', 'MarkerSize', 20, 'DisplayName', '$q_{start}$') % start configuration
    % plot(PP.sigma(1, len), PP.sigma(2, len), 'pentagram', 'MarkerSize', 20, 'DisplayName', '$q_{goal}$') % goal configuration
    % len1 = length(RB.r_lr);
    % AddXYData(plotObjLeftFoot, RB.r_lr(1, 1:2:len1), RB.r_lr(2, 1:2:len1))
    % AddXYData(plotObjRightFoot, RB.r_lr(1, 2:2:len1), RB.r_lr(2, 2:2:len1))
% end

% left and right footprints
plotObjLeftFoot = plot(rb{1}.r_lr(1, 1), rb{1}.r_lr(2, 1), 'square', DisplayName='left foot'); % Initial
plotObjRightFoot = plot(rb{1}.r_lr(1, 2), rb{1}.r_lr(2, 2), 'o', Color=[0 .5 0], DisplayName='right foot');
for i = 1 : length(rb{1}.r_lr)
    for j = 1 : length(rb)
        PP = pp{j}; RB = rb{j};
        if(mod(i, 2) == 1)
            AddXYData(plotObjLeftFoot, RB.r_lr(1, i), RB.r_lr(2, i)) % left feet print
        else
            AddXYData(plotObjRightFoot, RB.r_lr(1, i), RB.r_lr(2, i)) % right feet print
        end
    end
    % drawnow
end
SaveFig(fig)

%% functions
function [pp, rb, map] = CoordConversion(pp, rb)
    for i = 1 : length(rb)
        % Rotete Z-axis -90
        rotZMinus90 = [0 1; -1 0];
        for j = 1 : length(pp{i}.r)
            pp{i}.r(:, j) = rotZMinus90*pp{i}.r(:, j); 
        end
        for j = 1 : length(rb{i}.r_lr)
            rb{i}.r_lr(:, j) = rotZMinus90*rb{i}.r_lr(:, j);
        end
        XY = [pp{1}.map.XWorldLimits; pp{1}.map.YWorldLimits];
        XY(1, 2) = XY(1, 2);
        XY = rotZMinus90*XY;
        map.x = XY(1, :);
        map.y = XY(2, :);

        % Translate Y axis +6
        pp{i}.r(2, :) = pp{i}.r(2, :) + 6; 
        rb{i}.r_lr(2, :) = rb{i}.r_lr(2, :) + 6;  
        map.y = map.y + 6;
    end
end
function AddXYData(plotObj, XData, YData)
    plotObj.XData = cat(2, plotObj.XData, XData);
    plotObj.YData = cat(2, plotObj.YData, YData);
end