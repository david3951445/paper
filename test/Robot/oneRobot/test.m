close all
r2 = rb.tr.r{2}(1, :);
t2 = linspace(0, 1, length(r2));
figure; hold on
plot(t2, r2)
r3 = filloutliers(r2, 'pchip', 'movmedian', 10);
plot(t2, r3)
legend
