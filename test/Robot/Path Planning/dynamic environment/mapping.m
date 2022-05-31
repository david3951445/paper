function map = mapping(imageCropped)
%Return an occupancy map for each moment
% It's a simulation for real systems. In practice, the occupancy map for each moment
% can be construct by sensors on agents. 

% add obstale to map
DIM = size(imageCropped);
obstacle.pos = [speed.obstacle*i ceil(DIM(2)*2/3)]; % obstacle start position (left-top)
obstacle.size = [100 50]; % obstacle size, 20*10
obstacle.val = ones(obstacle.size(1), obstacle.size(2)); 
imageCropped(obstacle.pos(1) + (1:obstacle.size(1)), obstacle.pos(2) + (1:obstacle.size(2))) = obstacle.val;
% construct occupancyMap
imageNorm = double(imageCropped)/255;
imageOccupancy = 1 - imageNorm;
map = occupancyMap(imageOccupancy,20);
end

