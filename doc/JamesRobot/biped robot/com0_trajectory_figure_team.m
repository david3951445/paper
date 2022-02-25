clc;clear;close all
load('com0_trajectory_reference_model_team.mat');
%%
n = 12;
k = 1;
figure;
plot(tr,xr(k,:),'r',tr,xr(k+n,:),'g',tr,xr(k+2*n,:),'c',tr,xr(k+3*n,:),'m',tr,xr(k+4*n,:),'b',tr,xr(k+5*n,:),'k');
title('t-p_{x}');
legend('Robot_{1}','Robot_{2}','Robot_{3}','Robot_{4}','Robot_{5}','Robot_{6}');
grid on

k = 2;
figure;
plot(tr,xr(k,:),'r',tr,xr(k+n,:),'g',tr,xr(k+2*n,:),'c',tr,xr(k+3*n,:),'m',tr,xr(k+4*n,:),'b',tr,xr(k+5*n,:),'k');
title('t-p_{y}');
legend('Robot_{1}','Robot_{2}','Robot_{3}','Robot_{4}','Robot_{5}','Robot_{6}');
grid on

k = 3;
figure;
plot(tr,xr(k,:),'r',tr,xr(k+n,:),'g',tr,xr(k+2*n,:),'c',tr,xr(k+3*n,:),'m',tr,xr(k+4*n,:),'b',tr,xr(k+5*n,:),'k');
title('t-v_{x}');
legend('Robot_{1}','Robot_{2}','Robot_{3}','Robot_{4}','Robot_{5}','Robot_{6}');
grid on

k = 4;
figure;
plot(tr,xr(k,:),'r',tr,xr(k+n,:),'g',tr,xr(k+2*n,:),'c',tr,xr(k+3*n,:),'m',tr,xr(k+4*n,:),'b',tr,xr(k+5*n,:),'k');
title('t-v_{y}');
legend('Robot_{1}','Robot_{2}','Robot_{3}','Robot_{4}','Robot_{5}','Robot_{6}');
grid on

k = 5;
figure;
plot(tr,xr(k,:),'r',tr,xr(k+n,:),'g',tr,xr(k+2*n,:),'c',tr,xr(k+3*n,:),'m',tr,xr(k+4*n,:),'b',tr,xr(k+5*n,:),'k');
title('t-a_{x}');
legend('Robot_{1}','Robot_{2}','Robot_{3}','Robot_{4}','Robot_{5}','Robot_{6}');
grid on

k = 6;
figure;
plot(tr,xr(k,:),'r',tr,xr(k+n,:),'g',tr,xr(k+2*n,:),'c',tr,xr(k+3*n,:),'m',tr,xr(k+4*n,:),'b',tr,xr(k+5*n,:),'k');
title('t-a_{y}');
legend('Robot_{1}','Robot_{2}','Robot_{3}','Robot_{4}','Robot_{5}','Robot_{6}');
grid on

k = 7;
figure;
plot(tr,xr(k,:),'r',tr,xr(k+n,:),'g',tr,xr(k+2*n,:),'c',tr,xr(k+3*n,:),'m',tr,xr(k+4*n,:),'b',tr,xr(k+5*n,:),'k');
title('t-\phi');
legend('Robot_{1}','Robot_{2}','Robot_{3}','Robot_{4}','Robot_{5}','Robot_{6}');
grid on

k = 8;
figure;
plot(tr,xr(k,:),'r',tr,xr(k+n,:),'g',tr,xr(k+2*n,:),'c',tr,xr(k+3*n,:),'m',tr,xr(k+4*n,:),'b',tr,xr(k+5*n,:),'k');
title('t-\omega');
legend('Robot_{1}','Robot_{2}','Robot_{3}','Robot_{4}','Robot_{5}','Robot_{6}');
grid on

k = 9;
figure;
plot(tr,xr(k,:),'r',tr,xr(k+n,:),'g',tr,xr(k+2*n,:),'c',tr,xr(k+3*n,:),'m',tr,xr(k+4*n,:),'b',tr,xr(k+5*n,:),'k');
title('t-\alpha');
legend('Robot_{1}','Robot_{2}','Robot_{3}','Robot_{4}','Robot_{5}','Robot_{6}');
grid on

k = 10;
figure;
plot(tr,xr(k,:),'r',tr,xr(k+n,:),'g',tr,xr(k+2*n,:),'c',tr,xr(k+3*n,:),'m',tr,xr(k+4*n,:),'b',tr,xr(k+5*n,:),'k');
title('t-r');
legend('Robot_{1}','Robot_{2}','Robot_{3}','Robot_{4}','Robot_{5}','Robot_{6}');
grid on

k = 11;
figure;
plot(tr,xr(k,:),'r',tr,xr(k+n,:),'g',tr,xr(k+2*n,:),'c',tr,xr(k+3*n,:),'m',tr,xr(k+4*n,:),'b',tr,xr(k+5*n,:),'k');
title('t-v');
legend('Robot_{1}','Robot_{2}','Robot_{3}','Robot_{4}','Robot_{5}','Robot_{6}');
grid on

k = 12;
figure;
plot(tr,xr(k,:),'r',tr,xr(k+n,:),'g',tr,xr(k+2*n,:),'c',tr,xr(k+3*n,:),'m',tr,xr(k+4*n,:),'b',tr,xr(k+5*n,:),'k');
title('t-a');
legend('Robot_{1}','Robot_{2}','Robot_{3}','Robot_{4}','Robot_{5}','Robot_{6}');
grid on



figure;
plot(xr(1,:),xr(2,:),'r',xr(1+n,:),xr(2+n,:),'g',...
    xr(1+2*n,:),xr(2+2*n,:),'c',xr(1+3*n,:),xr(2+3*n,:),'m',...
    xr(1+4*n,:),xr(2+4*n,:),'b',xr(1+5*n,:),xr(2+5*n,:),'k');
title('P_{x}-P_{y}');
legend('Robot_{1}','Robot_{2}','Robot_{3}','Robot_{4}','Robot_{5}','Robot_{6}');
grid on

%% error 
% for i = 1:6
% figure;
% plot(tr,r_new(i,:)-xr(i,:));
% end