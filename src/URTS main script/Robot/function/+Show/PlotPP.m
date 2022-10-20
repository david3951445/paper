function PlotPP(pp)
    % Plot the occupancy map and path planning result
    fig = figure;
    show(pp.map);
    hold on;
    plot(pp.tree(:,1), pp.tree(:,2),'.-', 'DisplayName','RRT tree expansion'); % tree expansion
    % plot(pthObj.States(:,1), pthObj.States(:,2),'r-','LineWidth',2, 'DisplayName','path') % draw path
    plot(pp.sigma(1, :), pp.sigma(2, :), 'o', 'DisplayName', '$\sigma(t)$')
    plot(pp.r(1, :), pp.r(2, :), 'LineWidth', 2, 'DisplayName', '$\sigma''(t)$')
    len = length(pp.sigma);
    plot(pp.sigma(1, 1), pp.sigma(2, 1), 'square', 'MarkerSize', 20, 'DisplayName', '$q_{start}$')
    plot(pp.sigma(1, len), pp.sigma(2, len), 'pentagram', 'MarkerSize', 20, 'DisplayName', '$q_{goal}$')
    % len1 = 25;
    % plot(rb.r_lr(1, 1:2:len1), rb.r_lr(2, 1:2:len1), '-o', 'DisplayName', 'left foot')
    % plot(rb.r_lr(1, 2:2:len1), rb.r_lr(2, 2:2:len1), '-o', 'DisplayName', 'right foot')
    % plot(rb.zmp(1, :), rb.zmp(2, :), '-s', 'Displayname', 'ZMP trajectory')
    % plot(rb.CoM(1, :), rb.CoM(2, :), 'Displayname', 'CoM trajectory')
    axis equal
    % title('path of the robot \alpha_{1,2} using RRT algorithm')
    xlabel('x (m)'); ylabel('y (m)')
    legend('Interpreter','latex')%, 'FontSize', 20)
    SaveFig(fig)
end