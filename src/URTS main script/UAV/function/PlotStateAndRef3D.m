% 3D, r(t), state
function PlotStateAndRef3D(uav)
    figure

    r = uav.tr.r{1};
    plot3(r(1, :), r(2, :), r(3, :), DisplayName='reference');
    hold on

    X = uav.tr.x(uav.DIM_F+1, :) + r(1, :);
    Y = uav.tr.x(uav.DIM_F+2, :) + r(2, :);
    Z = uav.tr.x(uav.DIM_F+3, :) + r(3, :);
    plot3(X, Y, Z, DisplayName='state')

    grid on
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)')
    legend(Interpreter="latex", Location='best')

    % state, error, estimated state   
    Y_LABEL = {'x (m)', 'y (m)', 'z (m)', '\phi (rad)', '\theta (rad)', '\psi (rad)'};
end