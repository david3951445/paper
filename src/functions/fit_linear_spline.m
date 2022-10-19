function r3 = fit_linear_spline(r1, INTERP_DENSITY)
len1 = length(r1);
t1 = linspace(0,1,len1);

len2 = (len1-1)*INTERP_DENSITY(1)+1;
t2 = linspace(0,1,len2);
r2 = zeros(2, len2);
r2 = [
    interp1(t1, r1(1, :), t2);
    interp1(t1, r1(2, :), t2);
];

fx = fit(t2', r2(1,:)', 'SmoothingSpline');
fy = fit(t2', r2(2,:)', 'SmoothingSpline');
len3 = len2*INTERP_DENSITY(2);
t3 = linspace(0,1,len3);
r3 = [
    feval(fx, t3)';
    feval(fy, t3)';
];

end