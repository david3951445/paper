%Local motion planning
function PlotLMP(uav)
    r = uav.tr.r{1};
    fig = figure;
    
    plot3(uav.sigma(1, :), uav.sigma(2, :), uav.sigma(3, :), '^', DisplayName='$\sigma(t)$', MarkerSize=7)
    hold on
    
    plot3(r(1, :), r(2, :), r(3, :), DisplayName='$\sigma''(t)$');
    
    grid on
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)')
    legend(Interpreter="latex", Location='best')
    
    SaveFig(fig)
end