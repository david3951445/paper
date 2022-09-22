function rb = GRF(rb)
% left foot
% x = [1/6 3/6 5/6 1 7/6 8/6 9/6 10/6 11/6 2];
% y = [1 .9 1.2 .09 .01 0 0 0 .01 .09];
x = linspace(0, 1, 12);
y = [1 .95 .9 .95 1.1 .09 .01 0 0 0 .01 .09];
r = [
    x
    y*10
];
r0 = cat(2, [0; 0], r);
num_of_step_period = ceil((rb.tr.LEN-1)/(rb.INTERP_DENSITY*2)); % How many step
for i = 1 : num_of_step_period - 1
    r(1, :) = r(1, :) + i;
    r0 = cat(2, r0, r);
end

% plot(r0(1,:), r0(2,:))
r2 = fit_linear_spline(r0, [2 3]);

t = linspace(0, 1, length(r2));
fx = fit(t', r2(1,:)', 'SmoothingSpline');
fy = fit(t', r2(2,:)', 'SmoothingSpline');
t3 = linspace(0, 1, rb.tr.LEN);
r3 = [
    feval(fx, t3)';
    feval(fy, t3)';
];
r3 = r3(:, 1:rb.tr.LEN); % cut to fit the simulation trajectories
figure
plot(r3(1,:), r3(2,:))
axis equal
rb.tr.GRF{1} = r3(2,:); % left foot

end