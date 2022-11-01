function PlotTC(rb)
    % Plot tracking control result
    %
    % There are 4 figures:
    %   - Trajectory of state, estimated state, reference
    %   - Estimation of actuator and sensor fault
    %   - Control effort

    %% state, estimated state, reference
    fig = figure;
    DIM = rb.DIM_F;
    
    Layout = GetTiledlayout(DIM);
    r = rb.tr.r{1};
    for i = 1 : DIM % position
        nexttile
        hold on

        index = DIM + i;
        plot(rb.tr.t, rb.tr.x(index, :)+r(i, :))
        plot(rb.tr.t, rb.tr.xh(index, :)+r(i, :))
        plot(rb.tr.t, r(i, :))

        grid on
        ylabel(['$q_{' num2str(i) '} (rad)$'], 'Interpreter','latex')      
    end
    xlabel(Layout,'t (sec)')
    lg  = legend('state', 'estimated state', 'reference', NumColumns=3); 
    lg.Layout.Tile = 'north';
    % ylabel(Layout, 'rad')

    SaveFig(fig)

    %% Fa and Fs
    PlotFault(rb.tr.t, rb.tr.x, rb.tr.xh, rb.sys_a.begin, rb.sys_a.DIM, '1', 'm/s^2')
    PlotFault(rb.tr.t, rb.tr.x, rb.tr.xh, rb.sys_s.begin, rb.sys_s.DIM, '2', 'm')
    
    %% control u(t)
    fig = figure;
    DIM = size(rb.tr.u, 1);
    
    hold on
    for i = 1 : DIM
        index = i;
        plot(rb.tr.t, rb.tr.u(index, :), 'DisplayName', ['$u_{' num2str(i) '}$'], 'LineWidth', 1)    
    end
    xlabel('t (sec)')
    ylabel('$u (N\cdot m)$', 'Interpreter','latex')
    legend('Interpreter','latex','Location','southeast')

    SaveFig(fig)
end

%% Functions
function PlotFault(t, x, xh, begin, DIM, TITLE, unit)
    fig = figure   ;
    Layout = GetTiledlayout(DIM);

    for i = 1 : DIM
        nexttile
        hold on
        index = begin + i;
        plot(t, x(index, :))
        plot(t, xh(index, :))

        grid on
        ylabel(['$f_{' TITLE ',' num2str(i) '}(' unit ')$'], 'Interpreter','latex')      
    end
    xlabel(Layout,'t (sec)')
    lg  = legend('state', 'estimated state', NumColumns=3); 
    lg.Layout.Tile = 'north';

    SaveFig(fig)
end

function Layout = GetTiledlayout(DIM)
    % Obtain a suitable size   
    %-1 Method 1: Average. ex. 12 -> 4by3
    % div = divisors(DIM);
    % i = ceil((length(div))/2);
    % Layout = tiledlayout(DIM/div(i), div(i));

    %-2 Method 2: 2 col. ex. 12 -> 6by2
    Layout = tiledlayout(DIM/2, 2); % (row: DIM/2, col: 2)

    % Fill the figure window
    Layout.TileSpacing = 'tight';
    Layout.Padding = 'tight';
end