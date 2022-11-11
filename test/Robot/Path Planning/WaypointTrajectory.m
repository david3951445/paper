%A way to find pos, vel, acc by given waypoints.
% document:
%   https://www.mathworks.com/help/nav/ref/waypointtrajectory-system-object.html
clc; clear; close all;

%% A given waypoints
Waypoints = [
    0 1 1 0 1 2 3 4 5 5 4 4 3 2
    0 0 1 2 4 3 3 4 3 2 2 1 .5 0
];
Waypoints = [Waypoints; zeros(1, length(Waypoints))]';

%% Generated trajectory
TimeOfArrival = linspace(0, 10, length(Waypoints));
SampleRate = 1/(TimeOfArrival(2)-TimeOfArrival(1));
trajectory = waypointTrajectory(Waypoints,TimeOfArrival);
trajectory.SampleRate = 10*SampleRate;
% [position,orientation,velocity,acceleration,angularVelocity] = trajectory();

count = 1;
while ~isDone(trajectory)
   [currentPosition,~] = trajectory();
   position(:, count) = currentPosition';
%    plot(currentPosition(1),currentPosition(2),'bo')
   count = count + 1;
end

%% Plot
figure; hold on
plot(Waypoints(:, 1), Waypoints(:, 2), '-o')
plot(position(1, :), position(2, :), '-o')