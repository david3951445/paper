function PlotTC(ag)
    % Plot tracking control result
    %
    % There are 4 figures:
    %   - Trajectory of state, estimated state, reference
    %   - Estimation of actuator and sensor fault
    %   - Control effort

    %% state, estimated state, reference
    fig = figure;
    DIM = ag.DIM_F;
    
    Layout = GetTiledlayout(DIM);
    r = ag.tr.r{1};
    for i = 1 : DIM % position
        nexttile
        hold on
        grid on

        index = DIM + i;
        plot(ag.tr.t, ag.tr.x(index, :)+r(i, :))
        plot(ag.tr.t, ag.tr.xh(index, :)+r(i, :))
        plot(ag.tr.t, r(i, :))

        ylabel(['$x_{' num2str(i) '} (' ag.UNIT{i} ')$'], 'Interpreter','latex')      
    end
    xlabel(Layout,'t (sec)')
    lg  = legend('state', 'estimated state', 'reference', NumColumns=3); 
    lg.Layout.Tile = 'north';
    % ylabel(Layout, 'rad')

    SaveFig(fig)

    %% Fa and Fs
    ag.PlotFault('1')
    ag.PlotFault('2')
    
    %% control u(t)
    fig = figure;
    DIM = size(ag.tr.u, 1);
    
    hold on
    grid on
    for i = 1 : DIM
        index = i;
        plot(ag.tr.t, ag.tr.u(index, :), 'DisplayName', ['$u_{' num2str(i) '}$'], 'LineWidth', 1)    
    end
    xlabel('t (sec)')
    ylabel('$u (N\cdot m)$', 'Interpreter','latex')
    legend('Interpreter','latex','Location','southeast')

    SaveFig(fig)
end