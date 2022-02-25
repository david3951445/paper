clc;clear;close all
load leg_trajectory_test_v7.mat
% r = [pxr;pyr;thr;vxr;vyr;wr;axr;ayr;afar];
%%% generte transformation matrix %%%
% rotation matrix 
% for i = 1:length(tt)
%     eul(i,:) = [0 0 xr(3,i)];
% end
% rotmZYX = eul2rotm(eul);
% transformation vector 




% hf = 0.01; %% half of max height since the abs(z)
% c = 0.5; %% the constant multiply to v --> deteremine the gait length
% T = 1; %% time period of a single step
% num_step = 100/(2*T); %% the number iof step during walking 
% l = length(tt);
% th = linspace(-pi/2,3*pi/2+(2*pi*num_step),l);
% vr = (xr(4,:).^2+xr(5,:).^2).^(1/2);
% for i = 1:length(vr)
%     gait(:,i) = leg_gait(vr(i),th(i),T);
% end




figure;
plot(gait(1,:),gait(4,:));
title('x-z');
axis equal

%% 
% trans = [zeros(1,l);-gait(1,:);gait(4,:)];

l = length(tr);
trans = [gait(1,:);zeros(1,l);gait(4,:)];
for i = 1:length(tr)
    Tm(:,:,i) = [[eye(3) trans(:,i)]; 0 0 0 1];
end


eul = [pi/2 0 0];
rotmZYX = eul2rotm(eul);


ptrans = [0;0;-0.21]; % projected frame to base frame
for i = 1:length(tr)
    Tm(:,:,i) = [[rotmZYX ptrans]; 0 0 0 1]*Tm(:,:,i);
end
L3 = 0.0285;
L4 = 0.11;
L5 = 0.11;
for i = 1:length(tr)
    q(:,i) = leg_IK(Tm(:,:,i),L3,L4,L5);
end
figure;
plot(tr,q(1,:));
title('t-q1');
axis equal
figure;
plot(tr,q(2,:));
title('t-q2');
axis equal
figure;
plot(tr,q(3,:));
title('t-q3');
axis equal
figure;
plot(tr,q(4,:));
title('t-q4');
axis equal
