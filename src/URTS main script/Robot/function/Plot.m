function Plot(t, x, xh, j, DIM, TITLE)
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
    ylabel(['$f_{' TITLE ',' num2str(i) '} (rad)$'], 'Interpreter','latex')      
    legend('Interpreter','latex','Location','southeast')
% ylim([-2 2])
end
xlabel(Layout,'t (sec)')
end