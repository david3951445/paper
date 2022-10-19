function animation = drone_Animation(x,y,z,roll,pitch,yaw, r, fps)
% This Animation code is for QuadCopter. Written by Jitendra Singh 

%% Define design parameters
D2R = pi/180;
R2D = 180/pi;
b   = 0.6;   % the length of total square cover by whole body of quadcopter in meter
a   = b/3;   % the legth of small square base of quadcopter(b/4)
H   = 0.06;  % hight of drone in Z direction (4cm)
H_m = H+H/2; % hight of motor in z direction (5 cm)
r_p = b/4;   % radius of propeller
%% Conversions
ro = 45*D2R;                   % angle by which rotate the base of quadcopter
Ri = [cos(ro) -sin(ro) 0;
      sin(ro) cos(ro)  0;
       0       0       1];     % rotation matrix to rotate the coordinates of base 
base_co = [-a/2  a/2 a/2 -a/2; % Coordinates of Base 
           -a/2 -a/2 a/2 a/2;
             0    0   0   0];
base = Ri*base_co;             % rotate base Coordinates by 45 degree 

to = linspace(0, 2*pi);
xp = r_p*cos(to);
yp = r_p*sin(to);
zp = zeros(1,length(to));
%% Define Figure plot
 fig1 = figure('pos', [0 50 800 600]);
 hg   = gca;

 
 

plot3(r(1, :),r(2, :),r(3, :),'DisplayName', 'reference r(t)', 'LineWidth',1.5); % plot ref
axis equal;
grid on;
%  view(68,50);
view(150,20);
xlim([-.5 10.5]); ylim([-.5 10.5]); zlim([-1.5 1.5]);
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');
hold(gca, 'on');
legend('Interpreter','latex','Location','best');

%% Design Different parts
% design the base square
 drone(1) = patch([base(1,:)],[base(2,:)],[base(3,:)],'r');
 drone(2) = patch([base(1,:)],[base(2,:)],[base(3,:)+H],'r');
 alpha(drone(1:2),0.7);
% design 2 parpendiculer legs of quadcopter 
 [xcylinder ycylinder zcylinder] = cylinder([H/2 H/2]);
 drone(3) =  surface(b*zcylinder-b/2,ycylinder,xcylinder+H/2,'facecolor','b');
 drone(4) =  surface(ycylinder,b*zcylinder-b/2,xcylinder+H/2,'facecolor','b') ; 
 alpha(drone(3:4),0.6);
% design 4 cylindrical motors 
 drone(5) = surface(xcylinder+b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
 drone(6) = surface(xcylinder-b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
 drone(7) = surface(xcylinder,ycylinder+b/2,H_m*zcylinder+H/2,'facecolor','r');
 drone(8) = surface(xcylinder,ycylinder-b/2,H_m*zcylinder+H/2,'facecolor','r');
 alpha(drone(5:8),0.7);
% design 4 propellers
 drone(9)  = patch(xp+b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
 drone(10) = patch(xp-b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
 drone(11) = patch(xp,yp+b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
 drone(12) = patch(xp,yp-b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
 alpha(drone(9:12),0.3);

%% create a group object and parent surface
  combinedobject = hgtransform('parent',hg );
  set(drone,'parent',combinedobject);
%  drawnow

disp('animationing ...')
F = cell(1, length(x));
plotState = plot3(x(1:1),y(1:1),z(1:1), 'b:','DisplayName', 'state x(t)','LineWidth',1.5);
 for i = 1:length(x)
     disp(num2str(i))
     if mod(i, fps) == 0
          title(['t = ' num2str(i/fps)])
%         disp(['progress: ' num2str(i/length(x))*100 '%'])
     end
     
     plotState.XData = x(1:i);
     plotState.YData = y(1:i);
     plotState.ZData = z(1:i);
     
     translation = makehgtform('translate',...
                               [x(i) y(i) z(i)]);
     %set(combinedobject, 'matrix',translation);
     rotation1 = makehgtform('xrotate',(pi/180)*(roll(i)));
     rotation2 = makehgtform('yrotate',(pi/180)*(pitch(i)));
     rotation3 = makehgtform('zrotate',yaw(i));
     %scaling = makehgtform('scale',1-i/20);
     set(combinedobject,'matrix',...
          translation*rotation3*rotation2*rotation1);
      
      %movieVector(i) =  getframe(fig1); 
        %delete(b);
%      drawnow
   % pause(0.2);
   F{i} = getframe(gcf);
  end

 
 disp('making .avi ...')
%% save frame to .avi
writerObj = VideoWriter('myVideo.avi');
writerObj.FrameRate = fps;
open(writerObj);
 for i = 1 : length(F)
    writeVideo(writerObj, F{i});
 end
 close(writerObj);

 
