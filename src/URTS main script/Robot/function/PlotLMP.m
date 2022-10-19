function PlotLMP(pp, rb)
    % Plot local motion planning result
    fig = figure;
    hold on;
    len = 4;
    plot(pp.sigma(1, 1:len), pp.sigma(2, 1:len), '^', 'DisplayName', '$\sigma(t)$','MarkerSize', 7)
    len = 20;
    plot(pp.r(1, 1:len), pp.r(2, 1:len), 'LineWidth', 1, 'DisplayName', '$\sigma''(t)$')
    % len = length(pp.sigma);
    % plot(pp.sigma(1, 1), pp.sigma(2, 1), 'square', 'MarkerSize', 20, 'DisplayName', '$q_{start}$')
    % plot(pp.sigma(1, len), pp.sigma(2, len), 'pentagram', 'MarkerSize', 20, 'DisplayName', '$q_{goal}$')
    len = rb.tr.LEN;
    plot(rb.zmp(1, 1:len), rb.zmp(2, 1:len), 'Displayname', 'ZMP path', 'LineWidth', 1)
    plot(rb.CoM(1, 1:len), rb.CoM(2, 1:len), 'Displayname', 'CoM path', 'LineWidth', 1)
    len1 = 25;
    plot(rb.r_lr(1, 1:2:len1), rb.r_lr(2, 1:2:len1), 'o', 'DisplayName', 'left footholds path', 'LineWidth', 1)
    plot(rb.r_lr(1, 2:2:len1), rb.r_lr(2, 2:2:len1), 'square', 'DisplayName', 'right footholds path', 'LineWidth', 1)
    axis equal
    % title('local motion planning')
    xlabel('x (m)'); ylabel('y (m)')
    legend('Interpreter','latex')%, 'FontSize', 20)
    % exportgraphics(fig,'lmp.png','Resolution',500)
    SaveFig(fig)
end