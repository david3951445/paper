% Plot tracking control result
%
% There are 4 figures:
%   - Trajectory of state, estimated state, reference
%   - Estimation of actuator and sensor fault
%   - Control effort

function PlotTC(ag)
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

        ylabel(['$x_{' num2str(i) '} (' ag.UNIT{i} ')$'], Interpreter='latex')
        
        if strcmp(ag.FILE_NAME, 'UAV_AGENTmodel') % UAV
            if (i>3) % psi, theta, phi
                ylim([-2 2])
            end
        end
    end
    xlabel(Layout,'t (sec)')
    lg  = legend('state $x(t)$', 'estimated state $\hat{x}(t)$ ', 'reference $r(t)$', NumColumns=3, Interpreter='latex'); 
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
        plot(ag.tr.t, ag.tr.u(index, :), 'DisplayName', ['$u_{' num2str(i) '}$'])    
    end
    xlabel('t (sec)')
    ylabel('$u (N\cdot m)$', 'Interpreter','latex')
    legend(Interpreter='latex',Location='southeast')

    SaveFig(fig)
end