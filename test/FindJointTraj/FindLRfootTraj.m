function [r_l, r_lh, frame] = FindLRfootTraj(r_l, splineDensity, FIRST_HIGHEST_FOOT, len2, height_feet)
%Find trajectory of left (right) foot

len2_1 = len2*2-1;
r_l = [r_l; zeros(1, len2)]; % add z-component
r_l(3, FIRST_HIGHEST_FOOT:2:len2) = height_feet; % set highest point of foot in a footstep

%% Let foot stand on ground when another is moving
% Left foot stand still when right foot is stepping. Thus, adding 2 point
% between foothold to let left foot "stand longer"
r_l = repelem(r_l, 1, 2);
r_l = r_l(:, 1:len2_1);
foothold = FIRST_HIGHEST_FOOT*2:4:len2_1;
r_l(:, foothold) = r_l(:, 1+foothold);

%% spline
len3 = (len2-1)*splineDensity.zmp+1;
xx = linspace(0, 1, len3);
t1 = linspace(0, 1, len2_1);
r_lh = [
    spline(t1, [0 r_l(1, :) 0], xx);
    spline(t1, [0 r_l(2, :) 0], xx);
    spline(t1, [0 r_l(3, :) 0], xx);
];

%% The results of spline between same value (footholds) isn't the same. Thus, correct it manually.
l = splineDensity.zmp/2;
foothold = (foothold/4-1)*(4*l)+(l*(4-1)+1);
for i = 1 : splineDensity.zmp-1
    r_lh(:, i+foothold) = r_lh(:, foothold);
end
% special case on start and end of trajectory
if FIRST_HIGHEST_FOOT == 2 % this foot start from ground and end from highest
    r_lh(:, 1:l) = repmat(r_lh(:, 1), 1, l);
else
    r_lh = r_lh(:, 1:len3);
end

%% find frames (orientation) of sole of left-right foot
dr_lh = my_diff(r_lh(1:2, :));
for i = 1 : len3
    frame(:, i) = [0 0 atan2(dr_lh(2, i), dr_lh(1, i))]';
end
for i = 1 : splineDensity.zmp-1
    frame(:, i+foothold) = frame(:, foothold);
end
% special case on start and end of trajectory
if FIRST_HIGHEST_FOOT == 2 % this foot start from ground and end from highest
    frame(:, 1:l) = repmat(frame(:, l+1), 1, l);
else
    frame = frame(:, 1:len3);
end

end