function Plot(t, x, xh, j, DIM, TITLE)
figure
Layout = tiledlayout(DIM, 1);
for i = 1 : DIM
    nexttile
    hold on
    index = j + i;
    plot(t, x(index, :), 'Displayname', 'state', 'LineWidth', 3)
    plot(t, xh(index, :), 'DisplayName', 'estimated', 'LineWidth', 3)
%     plot(t, x(index, :)-xh(index, :), 'Displayname', 'error', 'LineWidth', 3)
    grid on
    legend
    ylim([-1 1])
    title(['v' TITLE '_' num2str(i)])
end
title(Layout, ['F' TITLE])
end