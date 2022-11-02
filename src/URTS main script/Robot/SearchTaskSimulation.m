% Simulation of team1 for all search task
clc; clear; close all
addpath(genpath('../../../src'))

%% Path of UAV
uav = UAV_AGENTmodel();
factor = 250; % Sample rate, sample once for "factor" times. For speed up the simulation
fps = 1/uav.tr.dt;
fps = fps/factor;

uav.tr.r{1} = uav.tr.r{1}(:, 1:factor:length(uav.tr.r{1}));
e = uav.tr.x(uav.DIM_F + (1:uav.DIM_F), :);
e = e(:, 1:factor:length(e));
xUAV = e + uav.tr.r{1};

%% Path of robots
startPos = [0.75 0 0; 0.75 6.5 0]; % (x, y, z), Start position of first robot
Na = 5; % number of agents in a team
rb = cell(1, Na-1);
pp = rb;
for i = 1 : Na-1
    disp(['Path planning, ' num2str(i) '-th robot'])
    rb{i} = Robot(); % Load the simulation result
    pp{i} = PathPlanning(startPos(1, :), startPos(2, :)); % Find task space ref in a map using RRT
    % Since we want r_lr only, disable the Z2C and IK part
    rb{i}.EXE_Z2C = 0; % Do not calculate Z2C (ZMP to CoM)
    rb{i}.EXE_IK = 0; % Do not calculate IK
    rb{i} = rb{i}.Ref2Config(pp{i}.r); % Calculate r_lr % Left and right footholds path

    startPos(:, 1) = startPos(:, 1) + 1.5; % x += 1.5
end

% State
r = rb{1}.tr.r{1}'; % reference
x = rb{1}.tr.x(12 + (1:12), :);

config = r(1:factor:length(r), :); % For simulatiom speed, compress the array by a factor
config = cat(2, config(:, 1:2:11), config(:, 2:2:12)); % To match the rigidbodytree config format, 1,2,...,12 -> 1,3,...11,2,4,...,12

transform = Config2LRSupportFeetTransform(rb{1}.rbtree, config, rb{1}.INTERP_DENSITY/factor); % Left and Right Support Feet Transformation

%% Coordinate conversion
for i = 1 : length(rb)
    rb{i}.r_lr = CoordConversion(rb{i}.r_lr); % Left and right feet path
    pp{i}.r = CoordConversion(pp{i}.r); % Path planing result
end

for i = 1 : length(transform) % Convert cell to matrix
    temp(:, i) = transform{i}(1:2, 4); % x, y position value
end
temp = CoordConversion(temp);
for i = 1 : length(transform) % Convert matrix to cell
    transform{i}(1:2, 4) = temp(:, i);
end

mapXY = [pp{1}.map.XWorldLimits; pp{1}.map.YWorldLimits];
mapXY = CoordConversion(mapXY);

%% Plot
[fig, axisRange] = Show.Senario(); % Fig setting, senario of search task in a team
Show.Area([axisRange(1, :); axisRange(2, :); axisRange(3, :)]) % Search area
Show.Map(mapXY(1, :), mapXY(2, :)) % Robot occupancy map

% create a drone object
drone = GetDrone();
combinedobject = hgtransform('parent', gca);
set(drone, 'parent', combinedobject);

% Initialize
plot3(uav.tr.r{1}(1, :), uav.tr.r{1}(2, :), uav.tr.r{1}(3, :),'DisplayName', 'UAV reference', 'LineWidth',1.5); % UAV ref
plotState = plot3(xUAV(1, 1), xUAV(2, 1), xUAV(3, 1), 'b:','DisplayName', 'UAV state','LineWidth',1.5); % UAV state
plotObjLeftFoot = plot(rb{1}.r_lr(1, 1), rb{1}.r_lr(2, 1), 'square', DisplayName='left footholds'); % Robot left footholds ref
plotObjRightFoot = plot(rb{1}.r_lr(1, 2), rb{1}.r_lr(2, 2), 'o', Color=[0 .5 0], DisplayName='right footholds'); % Robot right footholds ref

% Find max length of trajectory in agents
maxLen = 0;
for i = 1 : length(rb) % robots
    if length(rb{i}.r_lr) > maxLen
        maxLen = length(rb{i}.r_lr);
    end
end
maxLen = max(maxLen, length(xUAV)); % UAV

% Add data to plot (Inorder to animate)
F = cell(1, maxLen); % Store frames
indexUAV = 1; 
myWaitbar = waitbar(0, 'Plot trajectory...');
title('T = 0')
for i = 1 : maxLen
    waitbar(i/maxLen, myWaitbar);

    for j = 1 : rb{1}.INTERP_DENSITY/factor
        if indexUAV <= length(xUAV)
            % UAV state
            plotState.XData = cat(2, plotState.XData, xUAV(1, indexUAV));
            plotState.YData = cat(2, plotState.YData, xUAV(2, indexUAV));
            plotState.ZData = cat(2, plotState.ZData, xUAV(3, indexUAV));

            % UAV object
            translation = makehgtform('translate', [xUAV(1, indexUAV) xUAV(2, indexUAV) xUAV(3, indexUAV)]);
            %set(combinedobject, 'matrix',translation);
            rotation1 = makehgtform('xrotate',xUAV(4, indexUAV));
            rotation2 = makehgtform('yrotate',xUAV(5, indexUAV));
            rotation3 = makehgtform('zrotate',xUAV(6, indexUAV));
            %scaling = makehgtform('scale',1-i/20);
            set(combinedobject, 'matrix', translation*rotation3*rotation2*rotation1);
        end
        % robots
        for k = 1 : length(rb)
            PP = pp{k}; RB = rb{k};
            if i <= length(RB.r_lr)
                if(mod(i, 2) == 1)
                    AddXYData(plotObjLeftFoot, RB.r_lr(1, i), RB.r_lr(2, i)) % left feet hold
                else
                    AddXYData(plotObjRightFoot, RB.r_lr(1, i), RB.r_lr(2, i)) % right feet hold
                end
            end
        end
        
        if (mod(indexUAV, fps) == 0)
            title(['T = ' num2str(indexUAV/fps)])
        end
        F{indexUAV} = getframe(gcf);
        indexUAV = indexUAV + 1;
        % drawnow
    end

end
close(myWaitbar)
SaveFig(fig)

% State
% p = poseplot("ENU", ScaleFactor=0.0002, MeshFileName='PlateSquareHoleSolid.stl', DisplayName='foothold paths');
% for i = 1 : length(transform)
%     p.Orientation = tform2rotm(transform{i});
%     p.Position = tform2trvec(transform{i});
%     drawnow
%     F{i} = getframe(gcf);
% end

% Plot sigma
% PP = pp{1}; RB = rb{1};
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

MakeVideo(F, fps, "data/searchTask")

%% functions
function newCoords = CoordConversion(coords)
    rotZMinus90 = [0 1; -1 0];
    for j = 1 : length(coords)
        coords(:, j) = rotZMinus90*coords(:, j); % Rotete Z-axis -90
        coords(2, j) = coords(2, j) + 6; % Translate Y axis +6
    end
    newCoords = coords;
end

function AddXYData(plotObj, XData, YData)
    plotObj.XData = cat(2, plotObj.XData, XData);
    plotObj.YData = cat(2, plotObj.YData, YData);
end