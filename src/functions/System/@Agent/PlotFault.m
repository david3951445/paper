function PlotFault(ag, TITLE)
    switch TITLE
        case '1' % actuator
            unit = '/s^2';
            sys = ag.sys_a;
        case '2' % sensor
            unit = '';
            sys = ag.sys_s;
    end

    fig = figure;
    Layout = GetTiledlayout(sys.DIM);

    for i = 1 : sys.DIM
        nexttile
        hold on
        index = sys.begin + i;
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