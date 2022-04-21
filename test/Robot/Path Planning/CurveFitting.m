clc; clear; close all
r_dc = [
    0 1 1 0 1 2 3 4 5 5 4 4 3 2
    0 0 1 2 4 3 3 4 3 2 2 1 .5 0
];
t = linspace(0,1,length(r_dc));
x0 = r_dc(1, :);
y0 = r_dc(2, :);
t3 = linspace(0,1,(length(r_dc)-1)*4+1);
x = interp1(t, x0, t3);
y = interp1(t, y0, t3);

options = fitoptions;
% options = fitoptions('Method','SmoothingSpline',...
%                      'SmoothingParam',1);
type = 'SmoothingSpline';
% type = 'cubicinterp';
% type = 'linearinterp';
f1 = fit(t3', x', type, options);
f2 = fit(t3', y', type, options);
t2 = linspace(0,1,length(r_dc)*15);
x2 = feval(f1,t2);
y2 = feval(f2,t2);

figure
plot(f1,t3,x)
figure; hold on
plot(x0,y0)
plot(x,y)
plot(x2,y2)
legend