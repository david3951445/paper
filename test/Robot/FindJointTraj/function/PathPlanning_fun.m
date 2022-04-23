% Old method
% function r = PathPlanning()
% %Find path in a map

% %% Path findind ...
% % r_dc = PathPlanningAlgorithm(map);
% %

% %% Construct a suitable path for robot
% SPLINE_DENSITY = 5; 
% % discrete waypoints for testing (on x-y plane)
% r_dc = [
%     0 0 1
%     0 1 1
% ]/5;
% % r_dc = [
% %     0 0 1 1 1 2 3 3 2
% %     0 1 1 2 3 3 3 2 2
% % ];
% % r_dc = [
% %     0 1 1 0 1 2 3 4 5 5 4 4 3 2
% %     0 0 1 2 4 3 3 4 3 2 2 1 .5 0
% % ];

% len1 = length(r_dc);
% t = linspace(0, 1, len1);
% len2 = len1*SPLINE_DENSITY;
% xx = linspace(0, 1, len2);
% % using spline fit discrete waypoints to generate the r(t)
% r = [
%     spline(t, [0 r_dc(1, :) 0], xx)
%     spline(t, [0 r_dc(2, :) 0], xx)
% ];

% end