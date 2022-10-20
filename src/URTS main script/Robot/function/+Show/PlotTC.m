function PlotTC(rb)
    % Plot tracking control result
    %
    % There are three figures:
    %   - Trajectory of state, estimated state, reference
    %   - Estimation of actuator and sensor fault
    %   - Control effort

    %% state, estimated state, reference
    fig = figure;
    DIM = rb.DIM_F;
    % div = divisors(DIM);
    % i = ceil((length(div))/2);
    % Layout = tiledlayout(DIM/div(i), div(i));
    Layout = tiledlayout(DIM/2, 2);
    Layout.TileSpacing = 'tight';
    Layout.Padding = 'tight';
    r = rb.tr.r{1};
    for i = 1 : DIM % position
        nexttile
        hold on
        index = DIM + i;
        plot(rb.tr.t, rb.tr.x(index, :)+r(i, :), 'DisplayName', 'state', 'LineWidth', 2)
        plot(rb.tr.t, rb.tr.xh(index, :)+r(i, :), 'DisplayName', 'estimated', 'LineWidth', 2)
        plot(rb.tr.t, r(i, :), 'DisplayName', 'reference', 'LineWidth', 2)
        grid on
        ylabel(['$q_{' num2str(i) '} (rad)$'], 'Interpreter','latex')      
        legend('Interpreter','latex','Location','southeast')
    end
    xlabel(Layout,'t (sec)')
    
    %% Fa and Fs
    Plot(rb.tr.t, rb.tr.x, rb.tr.xh, rb.sys_a.begin, rb.sys_a.DIM, '1', 'm/s^2')
    Plot(rb.tr.t, rb.tr.x, rb.tr.xh, rb.sys_s.begin, rb.sys_s.DIM, '2', 'm')

    %% control u(t)
    fig = figure;
    DIM = size(rb.tr.u, 1);
    % div = divisors(DIM);
    % i = ceil((length(div))/2);
    % Layout = tiledlayout(DIM/div(i), div(i));
    
    hold on
    for i = 1 : DIM
        % nexttile
        index = i;
        plot(rb.tr.t, rb.tr.u(index, :), 'DisplayName', ['$u_{' num2str(i) '}$'], 'LineWidth', 1)    
    end
    legend('Interpreter','latex','Location','southeast')
    xlabel("t")
    ylabel('$N\cdot m$', 'Interpreter','latex')
end

function Plot(t, x, xh, j, DIM, TITLE, unit)
    figure
    % obtain a suitable size
    Layout = tiledlayout(DIM/2, 2);
    for i = 1 : DIM
        nexttile
        hold on
        index = j + i;
        plot(t, x(index, :), 'Displayname', 'state', 'LineWidth', 1)
        plot(t, xh(index, :), 'DisplayName', 'estimated state', 'LineWidth', 1)
    %     plot(t, x(index, :)-xh(index, :), 'Displayname', 'error', 'LineWidth', 3)
        grid on
        ylabel(['$f_{' TITLE ',' num2str(i) '} (' unit ')$'], 'Interpreter','latex')      
        legend('Interpreter','latex','Location','southeast')
    % ylim([-2 2])
    end
    xlabel(Layout,'t (sec)')
end