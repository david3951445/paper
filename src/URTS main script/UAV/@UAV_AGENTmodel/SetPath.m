%Construct path and smoothed path
%
% path          : Some waypoints
% smoothed path : The reference r(t) = [xd, yd, zd, phid]^T

function uav = SetPath(uav)
    %-1 circle up
    % uav.qr  = zeros(4, uav.tr.LEN);
    % amp_z   = 0.9;
    % amp     = 0.8;
    % freg    = 1;
    % for i = 1 : uav.tr.LEN - 1
    %     uav.qr(:, i) = [
    %         amp*sin(freg*uav.tr.t(i))
    %         amp*cos(freg*uav.tr.t(i))
    %         amp_z*uav.tr.t(i) + 1
    %         0
    %     ];
    % end

    %-2 search task
    r1 = [
        0 0 0 0 .5          .5 .5 1 1 1        1.5 1.5 1.5 2 2     2
        0 .5 1 2 2          1 0 0 1 2          2 1 0 0 1           2 
        0 .025 .05 .05 .1   .1 .1 .2 .2 .2     .3 .3 .3 .3 .3      .3
    ]*5;
    r1(1:2, :) = r1(1:2, :)/2;
    r1(3, :) = r1(3, :) + 2;

    len1 = length(r1);
    t1 = linspace(0,1,len1);
    density = 5;
    len2 = (len1-1)*density+1;
    t2 = linspace(0,1,len2);
    r2 = zeros(3, len2);
    len3 = uav.tr.LEN;
    t3 = linspace(0,1,len3);
    r3 = zeros(3, len3);
    for i = 1 : 3
        r2(i, :) = interp1(t1, r1(i, :), t2);
        f2{i} = fit(t2', r2(i, :)', 'SmoothingSpline');
        r3(i, :) = feval(f2{i}, t3)';
    end
    uav.sigma = r1;
    uav.qr = cat(1, r3, zeros(1, length(r3))); % phi = 0
    uav.Save('qr');
end