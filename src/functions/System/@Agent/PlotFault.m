function PlotFault(ag, TITLE)
    switch TITLE
        case '1' % actuator
            unit = '/s^2';
        case '2' % sensor
            unit = '';
    end

    fig = figure;
    Layout = GetTiledlayout(ag.sys_a.DIM);

    for i = 1 : ag.sys_a.DIM
        nexttile
        hold on
        index = ag.sys_a.begin + i;
        plot(ag.tr.t, ag.tr.x(index, :))
        plot(ag.tr.t, ag.tr.xh(index, :))

        grid on
        ylabel(['$f_{' TITLE ',' num2str(i) '}(' ag.UNIT{i} unit ')$'], 'Interpreter','latex')      
    end
    xlabel(Layout,'t (sec)')
    lg  = legend('state', 'estimated state', NumColumns=3); 
    lg.Layout.Tile = 'north';

    SaveFig(fig)
end