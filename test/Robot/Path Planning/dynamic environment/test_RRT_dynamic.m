%Dynamic planning using plannerRRT()
%
% document:
%   https://www.mathworks.com/help/nav/ref/plannerrrt.html
%
% This method is not quite correct since the path can be discontinuous 

clc; clear; close all

pre_start = [0 0]; % agent location
speed.agent = 2; 
speed.obstacle = 8;
r = [];
% load map image
MAP = imread('map.png');
% make a video
v = VideoWriter('peaks.avi');
v.FrameRate = 10;
open(v);
% making frame
fig = figure;
fig.Visible = 'off';
N = 120;
M(N) = struct('cdata',[],'colormap',[]); % frame

disp(['path planning ..., len=' num2str(N)])
for i = 1 : N
    disp(['i=' num2str(i)])
    
    %% update map
    imageCropped = MAP(:,:,1); % origin map
    obstacle.size = [100 50]; % obstacle size, 20*10
    obstacle.val = ones(obstacle.size(1), obstacle.size(2));
    DIM = size(imageCropped);
    pos_y = speed.obstacle*i;
    if pos_y > DIM(1) - obstacle.size(1)
        pos_y = DIM(1) - obstacle.size(1);
    end
    obstacle.pos = [pos_y ceil(DIM(2)*2/3)]; % obstacle start position (left-top)        
    imageCropped(obstacle.pos(1) + (1:obstacle.size(1)), obstacle.pos(2) + (1:obstacle.size(2))) = obstacle.val; % put obstacle to map
    % construct occupancyMap
    imageNorm = double(imageCropped)/255;
    imageOccupancy = 1 - imageNorm;
    map = occupancyMap(imageOccupancy, 20);
    % validator setup
    ss = stateSpaceDubins;
    ss.MinTurningRadius = 0.2;
    ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
    sv = validatorOccupancyMap(ss);
    sv.ValidationDistance = 0.1;
    sv.Map = map;

    %% Path planning
    if CC(sv, r) || i == 1 % path exist collision or first time instant
        % planner setup
        planner = plannerRRTStar(ss,sv);
        planner.MaxConnectionDistance = .5*5;
        % solve
        start = [pre_start, 0];
        goal = [40, 30, 0];
        rng(100,'twister'); % repeatable result
        [pthObj, solnInfo] = planner.plan(start,goal);
        r = pthObj.States(:, 1:2);
        r = post_processing(pthObj.States(:, 1:2)', [2 2])';
    end
    
    show(map)
    hold on
    plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-'); % tree expansion
    plot(r(:,1), r(:,2),'r-','LineWidth',2) % draw path
    plot(pre_start(1), pre_start(2), 's')
    plot(goal(1), goal(2), 'p')
    title(['i=' num2str(i)])
    drawnow
    
    M(i) = getframe(fig);
    writeVideo(v, M(i))
    
    if size(r, 1) == 1 % reach goal
        disp('reach the goal')
        break
    else
        speed_ = speed.agent;
        if speed_ > size(r, 1)
            speed_ = size(r, 1);
        end
        r = r(speed_:size(r,1), :); % agent move forward
        pre_start = r(1, 1:2); % current location
    end 
end

% fig.Visible = 'on';
% movie(M, 1, 2)
close(v)