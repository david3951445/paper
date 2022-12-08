% Simulation of team 1~3 for all search task
clc; clear; close all
addpath(genpath('../../../src'))

%% Path of UAV
uav = UAV_AGENTmodel();
uav.PATH  = ['../UAV/data/' uav.FILE_NAME]; % path of saved data
uav = uav.Load(); % load old data

factor = 250; % Sample rate, sample once for "factor" times. For speed up the simulation
fps = 1/uav.tr.dt/factor; % fps for output video

% Down sampling for state and reference state
uav.tr.r{1} = uav.tr.r{1}(:, 1:factor:length(uav.tr.r{1})); 
e = uav.tr.x(uav.DIM_F + (1:uav.DIM_F), :);
e = e(:, 1:factor:length(e));
xUAV = e + uav.tr.r{1};
    
%% Path of robots
startPos = [
    0.75 0 0    % Start postion
    0.75 6.5 0  % Goal position
]; % (x, y, z)
Na = 5; % Number of agents in a team
Nr = (Na-1)*3; % Number of robots

rb = cell(1, Na-1);
pp = cell(1, Na-1);
for i = 1 : Nr
    disp(['Path planning, ' num2str(i) '-th robot'])

    rb{i} = Robot();
    rb{i}.PATH  = ['../Robot/data/' rb{i}.FILE_NAME]; % path of saved data
    rb{i} = rb{i}.Load(); % load old data

    pp{i} = PathPlanning(startPos(1, :), startPos(2, :)); % Find task space ref in a map using RRT

    % Since we want r_lr only, disable the Z2C and IK part
    rb{i}.EXE_Z2C = 0; % Do not calculate Z2C (ZMP to CoM)
    rb{i}.EXE_IK = 0; % Do not calculate IK
    rb{i} = rb{i}.Ref2Config(pp{i}.r); % Calculate r_lr % Left and right footholds path

    startPos(:, 1) = startPos(:, 1) + 2; % x += 1.5
end

% State
r = rb{1}.tr.r{1}'; % reference
x = rb{1}.tr.x(12 + (1:12), :);

config = r(1:factor:length(r), :); % Down sampling
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
fig = figure;
axis equal
grid on
hold on
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')
legend('Interpreter','latex','Location','best')
view(-30,30);
axisRange = [mapXY(1, :); flip(mapXY(2, :)); 0 4];
xlim(axisRange(1, :)); ylim(axisRange(2, :)); zlim(axisRange(3, :));
% [fig, axisRange] = PlotSenario(); % Fig setting, senario of search task in a team
% PlotArea([axisRange(1, :); axisRange(2, :); axisRange(3, :)]) % Search area
PlotMap(mapXY(1, :), mapXY(2, :)) % Robot occupancy map

% % Create a drone object
% drone = GetDrone();
% combinedobject = hgtransform('parent', gca);
% set(drone, 'parent', combinedobject);

% % Initialize
% plot3(uav.tr.r{1}(1, :), uav.tr.r{1}(2, :), uav.tr.r{1}(3, :),'DisplayName', 'UAV reference', 'LineWidth',1.5); % UAV ref
% plot3(xUAV(1, :), xUAV(2, 1), xUAV(3, 1), 'b:','DisplayName', 'UAV state','LineWidth',1.5); % UAV state

for i = 1 : Nr
    PP = pp{i}; RB = rb{i};
    r_leftFoothold = rb{i}.r_lr(:, 1:2:length(rb{i}.r_lr));
    r_rightFoothold = rb{i}.r_lr(:, 2:2:length(rb{i}.r_lr));
    plot(PP.r(1, :), PP.r(2, :), '.', DisplayName='$\sigma''(t)$'); % smoothed path
    % plot(r_leftFoothold(1, :), r_leftFoothold(2, :), 'square', DisplayName='left footholds'); % Robot left footholds ref
    % plot(r_rightFoothold(1, :), r_rightFoothold(2, :), 'o', Color=[0 .5 0], DisplayName='right footholds'); % Robot right footholds ref
end

% % UAV object
% translation = makehgtform('translate', [xUAV(1, indexUAV) xUAV(2, indexUAV) xUAV(3, indexUAV)]);
% %set(combinedobject, 'matrix',translation);
% rotation1 = makehgtform('xrotate',xUAV(4, indexUAV));
% rotation2 = makehgtform('yrotate',xUAV(5, indexUAV));
% rotation3 = makehgtform('zrotate',xUAV(6, indexUAV));
% %scaling = makehgtform('scale',1-i/20);
% set(combinedobject, 'matrix', translation*rotation3*rotation2*rotation1);

SaveFig(fig)