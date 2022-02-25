clc;clear;
%% the method from nctu robotic (generate refernece input)%%% 
%% set start ponit, way ponit, end point 
A = [1,1];
B = [3,2];
C = [4.5,1];
D = [6,1.7];
E = [6,3.5];
F = [8,5];
%% trajectory design design
T = 20;
h = 0.0005; % virtual sample time --> deal with RungeKutta
[pxr,vxr,axr] = com0_planning(A(1),B(1),C(1),D(1),E(1),F(1),T,h);
[pyr,vyr,ayr] = com0_planning(A(2),B(2),C(2),D(2),E(2),F(2),T,h);
% thr = atan2(vyr,vxr);
thr = atan2(diff(pyr),diff(pxr));
thr = [thr(1),thr];
wr = diff(thr)/h;
wr = [wr(1),wr];
afar = diff(wr)/h;
afar = [afar(1),afar];
%　r in polar coordinate 
Rr = sqrt(pxr.^2 + pyr.^2); % the distance from {base} origin to {body} origin
Vr = sqrt(vxr.^2 + vyr.^2); % the refernece velocity of com0 
%% new frame (to plot the direction of the point)
% dir = [1;0];
% for i = 1:length(theta_d)
%     dir_bar(:,i) = [cosd(theta_d(i)) -sind(theta_d(i)); sind(theta_d(i)) cosd(theta_d(i))]*dir;
% end
% p = [px;py];
% p_dir = p + 0.1*dir_bar;

save com0_trajectory.mat
