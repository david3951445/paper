%example of using plannerRRT()
% document: https://www.mathworks.com/help/nav/ref/plannerrrt.html
% This is modified from the related MATLAB example but with user map
clc; clear; close all

%% construct map
X = imread('map.png');
imageCropped = X(:,:,1);
imageNorm = double(imageCropped)/255;
imageOccupancy = 1 - imageNorm;
map = occupancyMap(imageOccupancy,20);

imageCropped = X(:,:,1);
% add obstale to map
[DIM1, DIM2] = size(imageCropped);
obstacle.pos = [1 ceil(DIM2*2/3)]; % obstacle start position (left-top)
obstacle.size = [100 50]; % obstacle size, 20*10
obstacle.val = ones(obstacle.size(1), obstacle.size(2));
% obstacle.val = ones(10,20); 
imageCropped(obstacle.pos(1) + (1:obstacle.size(1)), obstacle.pos(2) + (1:obstacle.size(2))) = obstacle.val;
imageNorm = double(imageCropped)/255;
imageOccupancy = 1 - imageNorm;
Map = occupancyMap(imageOccupancy,20);
figure
show(Map)
%%
ss = stateSpaceDubins;
ss.MinTurningRadius = 0.2;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
sv = validatorOccupancyMap(ss);
sv.Map = map;
sv.ValidationDistance = 0.01;

planner = plannerRRTStar(ss,sv);
planner.MaxConnectionDistance = .5*5;
start = [0,0,0];
goal = [40,30,0];
rng(100,'twister'); % repeatable result
[pthObj, solnInfo] = planner.plan(start,goal);
figure
show(map)
hold on
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-'); % tree expansion
plot(pthObj.States(:,1), pthObj.States(:,2),'r-','LineWidth',2) % draw path
length(pthObj.States(:,1))