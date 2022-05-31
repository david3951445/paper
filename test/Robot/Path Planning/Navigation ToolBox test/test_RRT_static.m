clc; clear; close all

%% construct map
X = imread('map.png');
imageCropped = X(:,:,1);
imageNorm = double(imageCropped)/255;
imageOccupancy = 1 - imageNorm;
map = occupancyMap(imageOccupancy,20);

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
show(map);
hold on;
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-'); % tree expansion
plot(pthObj.States(:,1), pthObj.States(:,2),'r-','LineWidth',2) % draw path
length(pthObj.States(:,1))