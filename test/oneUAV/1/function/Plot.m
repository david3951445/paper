function Plot(tr)
% figure('units','normalized','outerposition',[0 0 1 1])
state = 1 : size(tr.x, 1);
for i = 1 : length(state)
    figure(i)
%     subplot(1, 2, i);
    plot(tr.t, tr.x(state(i), :), tr.t, tr.xr(state(i), :));
    title(['x_{' num2str(state(i)) '}']); legend("x", "x_r");
    xlabel("t"); ylim([-15 15])
end

end