clc; clear; close all

%% construct map
X = imread('map.png');
imageCropped = X(:,:,1);
imageNorm = double(imageCropped)/255;
imageOccupancy = 1 - imageNorm;
map = occupancyMap(imageOccupancy,20);

%%
sv = validatorOccupancyMap;
sv.Map = map;
% planner = plannerHybridAStar(sv,'MinTurningRadius',2,'MotionPrimitiveLength',3);
planner = plannerAStarGrid(map);
start = [1,1];
goal = [600,800];
refpath = planner.plan(start,goal);
show(planner)