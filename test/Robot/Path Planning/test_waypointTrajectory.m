%A way to find pos, vel, acc by given waypoints.
% using matlab toolbox "waypointTrajectory()"
clc; clear; close all;
Waypoints = [
    0 1 1 0 1 2 3 4 5 5 4 4 3 2
    0 0 1 2 4 3 3 4 3 2 2 1 .5 0
];
figure; hold on
Waypoints = [Waypoints; zeros(1, length(Waypoints))]';
plot(Waypoints(:, 1), Waypoints(:, 2), '-o')
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
position
plot(position(1, :), position(2, :), '-o')