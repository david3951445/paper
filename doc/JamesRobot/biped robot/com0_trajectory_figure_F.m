figure;
plot(t,r(1,:),'b--',t,r(13,:),'k--',tr,xr(1,:),'r',tr,xr(13,:),'g');
title('t-p_{x}');
legend('Reference input F_{1}','Reference input F_{2}','Robot_{1}','Robot_{2}');
grid on

figure;
plot(t,r(2,:),'b--',t,r(14,:),'k--',tr,xr(2,:),'r',tr,xr(14,:),'g');
title('t-p_{y}');
legend('Reference input F_{1}','Reference input F_{2}','Robot_{1}','Robot_{2}');
grid on

figure;
plot(t,r(3,:),'b--',t,r(15,:),'k--',tr,xr(3,:),'r',tr,xr(15,:),'g');
title('t-v_{x}');
legend('Reference input F_{1}','Reference input F_{2}','Robot_{1}','Robot_{2}');
grid on

figure;
plot(t,r(4,:),'b--',t,r(16,:),'k--',tr,xr(4,:),'r',tr,xr(16,:),'g');
title('t-v_{y}');
legend('Reference input F_{1}','Reference input F_{2}','Robot_{1}','Robot_{2}');
grid on

figure;
plot(t,r(5,:),'b--',t,r(17,:),'k--',tr,xr(5,:),'r',tr,xr(17,:),'g');
title('t-a_{x}');
legend('Reference input F_{1}','Reference input F_{2}','Robot_{1}','Robot_{2}');
grid on

figure;
plot(t,r(6,:),'b--',t,r(18,:),'k--',tr,xr(6,:),'r',tr,xr(18,:),'g');
title('t-a_{y}');
legend('Reference input F_{1}','Reference input F_{2}','Robot_{1}','Robot_{2}');
grid on

figure;
plot(t,r(7,:),'b--',t,r(19,:),'k--',tr,xr(7,:),'r',tr,xr(19,:),'g');
title('t-\phi');
legend('Reference input F_{1}','Reference input F_{2}','Robot_{1}','Robot_{2}');
grid on

figure;
plot(t,r(8,:),'b--',t,r(20,:),'k--',tr,xr(8,:),'r',tr,xr(20,:),'g');
title('t-\omega');
legend('Reference input F_{1}','Reference input F_{2}','Robot_{1}','Robot_{2}');
grid on

figure;
plot(t,r(9,:),'b--',t,r(21,:),'k--',tr,xr(9,:),'r',tr,xr(21,:),'g');
title('t-\alpha');
legend('Reference input F_{1}','Reference input F_{2}','Robot_{1}','Robot_{2}');
grid on

figure;
plot(t,r(10,:),'b--',t,r(22,:),'k--',tr,xr(10,:),'r',tr,xr(22,:),'g');
title('t-r');
legend('Reference input F_{1}','Reference input F_{2}','Robot_{1}','Robot_{2}');
grid on

figure;
plot(t,r(11,:),'b--',t,r(23,:),'k--',tr,xr(11,:),'r',tr,xr(23,:),'g');
title('t-v');
legend('Reference input F_{1}','Reference input F_{2}','Robot_{1}','Robot_{2}');
grid on

figure;
plot(t,r(12,:),'b--',t,r(24,:),'k--',tr,xr(12,:),'r',tr,xr(24,:),'g');
title('t-a');
legend('Reference input F_{1}','Reference input F_{2}','Robot_{1}','Robot_{2}');
grid on

figure;
plot(r(1,:),r(2,:),'b--',r(13,:),r(14,:),'k--',xr(1,:),xr(2,:),xr(13,:),xr(14,:));
title('P_{x}-P_{y}');
legend('Reference input F_{1}','Reference input F_{2}','Robot_{1}','Robot_{2}');
axis equal
grid on