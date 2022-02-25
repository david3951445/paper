clc;clear;close all
L = load('com0_trajectory_reference_model_l1_v2.mat');
F = load('com0_trajectory_reference_model_f1_v1.mat');

figure;
plot(L.r(1,:),L.r(2,:),'b--',L.r(13,:),L.r(14,:),'k--',...
    L.xr(1,:),L.xr(2,:),L.xr(13,:),L.xr(14,:),...
    F.r(1,:),F.r(2,:),'r--',F.r(13,:),F.r(14,:),'g--',...
    F.xr(1,:),F.xr(2,:),F.xr(13,:),F.xr(14,:));
title('P_{x}-P_{y}');
grid on
axis equal