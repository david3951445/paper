%Simulation of team 1~3 for search task in UAV Robot Team System (URTS)
% - UAV
%   - Show reference and state
% - Robot
%   - Show path planning result and the left and right footprints

clc; clear; close all
addpath(genpath('../../../src'))

%% Parameters
% Execute the specific part to calculate new data or just load data
EXE_ROBOT_PATHPLANNING  = 0;
EXE_ROBOT_REF2CONFIG    = 0;

Na = 5; % Number of agents in a team
Nt = 3; % Number of teams in URTS
Nr = (Na-1)*3; % Number of robots

%% Load
% Load UAV_AGENTmodel class data
UAV = UAV_AGENTmodel();
UAV.PATH  = ['../UAV/data/' 'oneUAV_FaFs']; % path of saved data
UAV = UAV.Load(); % load old data
uav = cell(1, Na-1);
for i = 1 : Nr
    uav{i} = UAV;
end

% Load Robot class data
RB = Robot();
RB.PATH  = ['../Robot/data/' RB.FILE_NAME]; % path of saved data
RB = RB.Load(); % load old data
rb = cell(1, Na-1);
for i = 1 : Nr
    rb{i} = RB;
end

%% Sample rate for down sampling
DownsamplingFactor = 250;
fps = 1/uav{1}.tr.dt/DownsamplingFactor; % fps for output video

%% Path of UAV
% Down sampling for state and reference state
offset = [0 0 0 0 0 0]';
for i = 1 : Nt
    % Reference
    r_UAV{i} = uav{i}.tr.r{1};
    r_UAV{i} = r_UAV{i}(:, 1:DownsamplingFactor:length(r_UAV{i})) + offset;

    % State
    e = uav{i}.tr.x(uav{1}.DIM_F + (1:uav{1}.DIM_F), :);
    e = e(:, 1:DownsamplingFactor:length(e));
    x_UAV{i} = e + r_UAV{i};

    offset = offset + [0 -8.5 0 0 0 0]';
end

%% Path of robots
% Path planning
startPos = [    % (x, y, z)
    0.75 0 0    % Start postion
    0.75 6.5 0  % Goal position
]; 
disp('Path planning...')
if (EXE_ROBOT_PATHPLANNING)
    pp = cell(1, Na-1);
    for i = 1 : Nr
        disp([num2str(i) '-th robot'])
        pp{i} = PathPlanning(startPos(1, :), startPos(2, :)); % Find task space ref in a map using RRT
        startPos(:, 1) = startPos(:, 1) + 2; % x += 1.5
    end

    save(['data/' mfilename '.mat'], 'pp')
else
    pp = load(['data/' mfilename '.mat']).pp;
end

disp('Transfer task space path to robot joint space path...')
if (EXE_ROBOT_REF2CONFIG)
    for i = 1 : Nr
        disp([num2str(i) '-th robot'])
    
        % Since we want r_lr only, disable the Z2C and IK part
        rb{i}.EXE_Z2C = 0; % Do not calculate Z2C (ZMP to CoM)
        rb{i}.EXE_IK = 0; % Do not calculate IK
        rb{i} = rb{i}.Ref2Config(pp{i}.r); % Calculate r_lr % Left and right footholds path    
        r_lr{i} = rb{i}.r_lr;
    end
   
    save(['data/' mfilename '.mat'], 'r_lr', '-append')
else
    r_lr = load(['data/' mfilename '.mat']).r_lr;
    for i = 1 : Nr
        rb{i}.r_lr = r_lr{i};
    end
end

% State
% x = rb{1}.tr.x(12 + (1:12), :);

% Reference
r = rb{1}.tr.r{1}'; % reference
config = r(1:DownsamplingFactor:length(r), :); % Down sampling
config = cat(2, config(:, 1:2:11), config(:, 2:2:12)); % To match the rigidbodytree config format, 1,2,...,12 -> 1,3,...11,2,4,...,12
transform = Config2LRSupportFeetTransform(rb{1}.rbtree, config, rb{1}.INTERP_DENSITY/DownsamplingFactor); % Left and Right Support Feet Transformation

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
disp('Plotting...')

fig = figure;
axis equal
grid on
hold on
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
legend('Interpreter','latex','Location','best')
view(-50, 15);
axisRange = [mapXY(1, :); flip(mapXY(2, :)); 0 4] + [-1 1; -1 1; -1 1]*.1;
xlim(axisRange(1, :)); ylim(axisRange(2, :)); zlim(axisRange(3, :));
% [fig, axisRange] = PlotSenario(); % Fig setting, senario of search task in a team
areaRangeY = linspace(axisRange(2, 1), axisRange(2, 2), 4);
PlotArea([axisRange(1, :); areaRangeY(3:4); axisRange(3, :)], '$area_1$', 'g')
PlotArea([axisRange(1, :); areaRangeY(2:3); axisRange(3, :)], '$area_2$', 'b')
PlotArea([axisRange(1, :); areaRangeY(1:2); axisRange(3, :)], '$area_3$', 'r')
PlotMap(mapXY(1, :), mapXY(2, :)) % Robot occupancy map

% % Create a drone object
% drone = GetDrone();
% combinedobject = hgtransform('parent', gca);
% set(drone, 'parent', combinedobject);

% UAV
plotObjUAVState = plot3(x_UAV{1}(1, 1), x_UAV{1}(2, 1), x_UAV{1}(3, 1), '.', DisplayName=['states of UAVs']);
plotObjAgentStart = plot3(r_UAV{1}(1, 1), r_UAV{1}(2, 1), r_UAV{1}(3, 1), '^', MarkerSize=12, DisplayName=['$q_{start}$ of agents']);
N = length(r_UAV{1});
plotObjAgentGoal = plot3(r_UAV{1}(1, N), r_UAV{1}(2, N), r_UAV{1}(3, N), 'pentagram', MarkerSize=12, DisplayName=['$q_{goal}$ of agents']);
for i = 1 : Nt
    % Reference
    plot3(r_UAV{i}(1, :), r_UAV{i}(2, :), r_UAV{i}(3, :), DisplayName=['$\sigma''(t)$ of $\alpha_{' num2str(i) ', 1}$']);

    % State
    range = 1 : floor(length(x_UAV{i})/2);
    AddXYZData(plotObjUAVState, x_UAV{i}(1, range), x_UAV{i}(2, range), x_UAV{i}(3, range))

    % Start
    AddXYZData(plotObjAgentStart, r_UAV{i}(1, 1), r_UAV{i}(2, 1), r_UAV{i}(3, 1))

    % Goal
    N = length(r_UAV{i});
    AddXYZData(plotObjAgentGoal, r_UAV{i}(1, N), r_UAV{i}(2, N), r_UAV{i}(3, N))
end

% robot
% initial
plotObjLeftFoot = plot(rb{1}.r_lr(1, 1), rb{1}.r_lr(2, 1), 'square', DisplayName='left footholds of robots'); % Robot left footholds ref
plotObjRightFoot = plot(rb{1}.r_lr(1, 2), rb{1}.r_lr(2, 2), 'o', Color=[0 .5 0], DisplayName='right footholds of robots'); % Robot right footholds ref

for i = 1 : Nr
    i_ = floor((i-1)/(Na-1)) + 1;
    j_ = mod(i-1, (Na-1)) + 1;
    PP = pp{i}; RB = rb{i};
    r_leftFoothold = rb{i}.r_lr(:, 1:2:length(rb{i}.r_lr));
    r_rightFoothold = rb{i}.r_lr(:, 2:2:length(rb{i}.r_lr));

    % Reference
    plot(PP.r(1, :), PP.r(2, :), '-.', DisplayName=['$\sigma''(t)$ of $\alpha_{' num2str(i_) ', ' num2str(j_) '}$']); % smoothed path
    
    % Left and right footholds
    range = 1 : floor(length(r_leftFoothold)/2);
    AddXYData(plotObjLeftFoot, r_leftFoothold(1, range), r_leftFoothold(2, range))
    AddXYData(plotObjRightFoot, r_rightFoothold(1, range), r_rightFoothold(2, range))

    % Start
    AddXYZData(plotObjAgentStart, PP.r(1, 1), PP.r(2, 1), 0)

    % Goal
    N = length(PP.r);
    AddXYZData(plotObjAgentGoal, PP.r(1, N), PP.r(2, N), 0)
end

% % UAV object
% translation = makehgtform('translate', [x_UAV(1, index_UAV) x_UAV(2, index_UAV) x_UAV(3, index_UAV)]);
% %set(combinedobject, 'matrix',translation);
% rotation1 = makehgtform('xrotate',x_UAV(4, index_UAV));
% rotation2 = makehgtform('yrotate',x_UAV(5, index_UAV));
% rotation3 = makehgtform('zrotate',x_UAV(6, index_UAV));
% %scaling = makehgtform('scale',1-i/20);
% set(combinedobject, 'matrix', translation*rotation3*rotation2*rotation1);

% SaveFig(fig)

function AddXYData(plotObj, XData, YData)
    plotObj.XData = cat(2, plotObj.XData, XData);
    plotObj.YData = cat(2, plotObj.YData, YData);
end
function AddXYZData(plotObj, XData, YData, ZData)
    AddXYData(plotObj, XData, YData)
    plotObj.ZData = cat(2, plotObj.ZData, ZData);
end
