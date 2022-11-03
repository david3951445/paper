%Plot local motion planning result

function PlotLMP(pp, rb)
    fig = figure;
    hold on

    % Path and smoothed path
    len = 4;
    plot(pp.sigma(1, 1:len), pp.sigma(2, 1:len), '^', 'DisplayName', '$(\sigma_n)$','MarkerSize', 7)
    len = 20;
    plot(pp.r(1, 1:len), pp.r(2, 1:len), 'LineWidth', 1, 'DisplayName', '$(\sigma''_n)$')

    % Start and goal configuration
    % len = length(pp.sigma);
    % plot(pp.sigma(1, 1), pp.sigma(2, 1), 'square', 'MarkerSize', 20, 'DisplayName', '$q_{start}$')
    % plot(pp.sigma(1, len), pp.sigma(2, len), 'pentagram', 'MarkerSize', 20, 'DisplayName', '$q_{goal}$')

    % ZMP and CoM
    len = rb.tr.LEN;
    plot(rb.zmp(1, 1:len), rb.zmp(2, 1:len), 'Displayname', 'ZMP path', 'LineWidth', 1)
    plot(rb.CoM(1, 1:len), rb.CoM(2, 1:len), 'Displayname', 'CoM path', 'LineWidth', 1)

    % Left and right footprints
    len = 25;
    plot(rb.r_lr(1, 1:2:len), rb.r_lr(2, 1:2:len), 'o', 'DisplayName', 'left footholds path', 'LineWidth', 1)
    plot(rb.r_lr(1, 2:2:len), rb.r_lr(2, 2:2:len), 'square', 'DisplayName', 'right footholds path', 'LineWidth', 1)

    axis equal
    xlabel('X (m)')
    ylabel('Y (m)')
    legend('Interpreter','latex','Location', 'Best')%, 'FontSize', 20)
    SaveFig(fig)
end