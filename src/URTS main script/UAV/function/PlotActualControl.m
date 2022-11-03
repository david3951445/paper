%Plot actual control input

function PlotActualControl(uav)
    fig = figure;
    Layout = tiledlayout(2, 1);
    Layout.TileSpacing = 'tight';
    Layout.Padding = 'tight';
    
    name = {'$\tau_x$', '$\tau_y$', '$\tau_z$'};
    nexttile
    for i = 1 : 3
        hold on
        index = 3 + i;
        plot(uav.tr.t, uav.tr.u(index, :), DisplayName=name{i})
    end
    grid on
    ylabel('$torque(N\cdot m)$', Interpreter='latex')
    legend(Interpreter='latex', Location='best'); 

    nexttile
    plot(uav.tr.t(1:uav.tr.LEN-1), uav.tr.F(1:uav.tr.LEN-1), DisplayName='F') 
    grid on
    ylabel('$force(N)$', Interpreter='latex')
    legend(Interpreter='latex', Location='best');
    
    xlabel('t (sec)')
    SaveFig(fig)
end