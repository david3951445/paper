clc; clear; close all

h = figure;
Z = peaks;
surf(Z)
axis tight manual
ax = gca;
ax.NextPlot = 'replaceChildren';

loops = 40;
M(loops) = struct('cdata',[],'colormap',[]);

h.Visible = 'off';
for j = 1:loops
    X = sin(j*pi/10)*Z;
    surf(X,Z)
    drawnow
    M(j) = getframe;
end

h.Visible = 'on';
movie(M);