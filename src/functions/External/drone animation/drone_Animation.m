function animation = drone_Animation(x,y,z,roll,pitch,yaw, r, fps)
% This Animation code is for QuadCopter. Written by Jitendra Singh

%% Define Figure plot
fig1 = figure('pos', [0 50 800 600]);
plot3(r(1, :),r(2, :),r(3, :),'DisplayName', 'reference r(t)', 'LineWidth',1.5); % plot ref
axis equal;
grid on;
%  view(68,50);
view(-30,20);
axisRange = [-.5 6.5; -4 6.5; 0 4];
xlim(axisRange(1, :)); ylim(axisRange(2, :)); zlim(axisRange(3, :));
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');
hold(gca, 'on');
legend('Interpreter','latex','Location','best');

drone = GetDrone(); % Drone object
%% create a group object and parent surface
combinedobject = hgtransform('parent', gca);
set(drone, 'parent', combinedobject);

disp('animationing ...')
F = cell(1, length(x));
plotState = plot3(x(1:1),y(1:1),z(1:1), 'b:','DisplayName', 'state x(t)','LineWidth',1.5);
for i = 1:length(x)
    disp(num2str(i))

    plotState.XData = x(1:i);
    plotState.YData = y(1:i);
    plotState.ZData = z(1:i);

    translation = makehgtform('translate', [x(i) y(i) z(i)]);
    %set(combinedobject, 'matrix',translation);
    rotation1 = makehgtform('xrotate',roll(i));
    rotation2 = makehgtform('yrotate',pitch(i));
    rotation3 = makehgtform('zrotate',yaw(i));
    %scaling = makehgtform('scale',1-i/20);
    set(combinedobject, 'matrix', translation*rotation3*rotation2*rotation1);

    F{i} = getframe(gcf);
end


disp('making .avi ...')
%% save frame to .avi
writerObj = VideoWriter('data/myVideo.avi');
writerObj.FrameRate = fps;
open(writerObj);
for i = 1 : length(F)
    writeVideo(writerObj, F{i});
end
close(writerObj);
end