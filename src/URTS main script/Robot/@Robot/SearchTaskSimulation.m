clc; clear; close all

startGoal = [0.75 0 0; 0.75 6.5 0];
for i = 1 : 4
    rb{i} = Robot();
    pp{i} = PathPlanning(startGoal(1, :), startGoal(2, :)); % Find task space ref in a map using RRT
    rb{i}.EXE_Z2C = 0;
    rb{i}.EXE_IK = 0;
    rb{i} = rb2.Ref2Config(pp.r);

    startGoal(:, 1) = startGoal(:, 1) + 1.5;
end

