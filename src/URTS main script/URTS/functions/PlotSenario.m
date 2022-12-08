function [fig, axisRange] = PlotSenario()
    % Setting of figure() for senario in URTS
    fig = figure;
    axis equal
    grid on
    hold on
    xlabel('X(m)')
    ylabel('Y(m)')
    zlabel('Z(m)')
    legend('Interpreter','latex','Location','best')

    %-1 View with UAV
    view(-30,30);
    axisRange = [-.5 6.5; -1 6.5; 0 4];
    xlim(axisRange(1, :)); ylim(axisRange(2, :)); zlim(axisRange(3, :));
    %-2 Zoom in view
    % view(0,20);
    % axisRange = [-.3 .6; -1.1 .1; -.04 .06];
    % xlim(axisRange(1, :)); ylim(axisRange(2, :)); zlim(axisRange(3, :));
end