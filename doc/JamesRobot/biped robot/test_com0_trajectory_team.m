clc;clear;close all
F11 = load('com0_trajectory_reference_model_team1_follower1.mat');
F12 = load('com0_trajectory_reference_model_team1_follower2.mat');
F13 = load('com0_trajectory_reference_model_team1_follower3.mat');
F21 = load('com0_trajectory_reference_model_team2_follower1.mat');
F22 = load('com0_trajectory_reference_model_team2_follower2.mat');
F23 = load('com0_trajectory_reference_model_team2_follower3.mat');
F31 = load('com0_trajectory_reference_model_team3_follower1.mat');
F32 = load('com0_trajectory_reference_model_team3_follower2.mat');
F33 = load('com0_trajectory_reference_model_team3_follower3.mat');
L1 = load('com0_trajectory_reference_model_team1_leader.mat');
L2 = load('com0_trajectory_reference_model_team2_leader.mat');
L3 = load('com0_trajectory_reference_model_team3_leader.mat');
%%
tr = F11.tr;
Hf = (length(tr)-1)/2+2000;
End = length(tr);

figure;
plot(L1.xr(1,:),L1.xr(2,:),'b',L1.xr(1,1),L1.xr(2,1),'bo',L1.xr(1,Hf),L1.xr(2,Hf),'bo',L1.xr(1,End),L1.xr(2,End),'bo',...
    L2.xr(1,:),L2.xr(2,:),'k',L2.xr(1,1),L2.xr(2,1),'k^',L2.xr(1,Hf),L2.xr(2,Hf),'k^',L2.xr(1,End),L2.xr(2,End),'k^',...
    L3.xr(1,:),L3.xr(2,:),'r',L3.xr(1,1),L3.xr(2,1),'rd',L3.xr(1,Hf),L3.xr(2,Hf),'rd',L3.xr(1,End),L3.xr(2,End),'rd',...
    F11.xr(1,:),F11.xr(2,:),'b',F11.xr(1,1),F11.xr(2,1),'bo',F11.xr(1,Hf),F11.xr(2,Hf),'bo',F11.xr(1,End),F11.xr(2,End),'bo',...
    F12.xr(1,:),F12.xr(2,:),'b',F12.xr(1,1),F12.xr(2,1),'bo',F12.xr(1,Hf),F12.xr(2,Hf),'bo',F12.xr(1,End),F12.xr(2,End),'bo',...
    F13.xr(1,:),F13.xr(2,:),'b',F13.xr(1,1),F13.xr(2,1),'bo',F13.xr(1,Hf),F13.xr(2,Hf),'bo',F13.xr(1,End),F13.xr(2,End),'bo',...
    F21.xr(1,:),F21.xr(2,:),'k',F21.xr(1,1),F21.xr(2,1),'k^',F21.xr(1,Hf),F21.xr(2,Hf),'k^',F21.xr(1,End),F21.xr(2,End),'k^',...
    F22.xr(1,:),F22.xr(2,:),'k',F22.xr(1,1),F22.xr(2,1),'k^',F22.xr(1,Hf),F22.xr(2,Hf),'k^',F22.xr(1,End),F22.xr(2,End),'k^',...
    F23.xr(1,:),F23.xr(2,:),'k',F23.xr(1,1),F23.xr(2,1),'k^',F23.xr(1,Hf),F23.xr(2,Hf),'k^',F23.xr(1,End),F23.xr(2,End),'k^',...
    F31.xr(1,:),F31.xr(2,:),'r',F31.xr(1,1),F31.xr(2,1),'rd',F31.xr(1,Hf),F31.xr(2,Hf),'rd',F31.xr(1,End),F31.xr(2,End),'rd',...
    F32.xr(1,:),F32.xr(2,:),'r',F32.xr(1,1),F32.xr(2,1),'rd',F32.xr(1,Hf),F32.xr(2,Hf),'rd',F32.xr(1,End),F32.xr(2,End),'rd',...
    F33.xr(1,:),F33.xr(2,:),'r',F33.xr(1,1),F33.xr(2,1),'rd',F33.xr(1,Hf),F33.xr(2,Hf),'rd',F33.xr(1,End),F33.xr(2,End),'rd',...
    F11.xr(13,:),F11.xr(14,:),'b',F11.xr(13,1),F11.xr(14,1),'bo',F11.xr(13,Hf),F11.xr(14,Hf),'bo',F11.xr(13,End),F11.xr(14,End),'bo',...
    F12.xr(13,:),F12.xr(14,:),'b',F12.xr(13,1),F12.xr(14,1),'bo',F12.xr(13,Hf),F12.xr(14,Hf),'bo',F12.xr(13,End),F12.xr(14,End),'bo',...
    F13.xr(13,:),F13.xr(14,:),'b',F13.xr(13,1),F13.xr(14,1),'bo',F13.xr(13,Hf),F13.xr(14,Hf),'bo',F13.xr(13,End),F13.xr(14,End),'bo',...
    F21.xr(13,:),F21.xr(14,:),'k',F21.xr(13,1),F21.xr(14,1),'k^',F21.xr(13,Hf),F21.xr(14,Hf),'k^',F21.xr(13,End),F21.xr(14,End),'k^',...
    F22.xr(13,:),F22.xr(14,:),'k',F22.xr(13,1),F22.xr(14,1),'k^',F22.xr(13,Hf),F22.xr(14,Hf),'k^',F22.xr(13,End),F22.xr(14,End),'k^',...
    F23.xr(13,:),F23.xr(14,:),'k',F23.xr(13,1),F23.xr(14,1),'k^',F23.xr(13,Hf),F23.xr(14,Hf),'k^',F23.xr(13,End),F23.xr(14,End),'k^',...
    F31.xr(13,:),F31.xr(14,:),'r',F31.xr(13,1),F31.xr(14,1),'rd',F31.xr(13,Hf),F31.xr(14,Hf),'rd',F31.xr(13,End),F31.xr(14,End),'rd',...
    F32.xr(13,:),F32.xr(14,:),'r',F32.xr(13,1),F32.xr(14,1),'rd',F32.xr(13,Hf),F32.xr(14,Hf),'rd',F32.xr(13,End),F32.xr(14,End),'rd',...
    F33.xr(13,:),F33.xr(14,:),'r',F33.xr(13,1),F33.xr(14,1),'rd',F33.xr(13,Hf),F33.xr(14,Hf),'rd',F33.xr(13,End),F33.xr(14,End),'rd');
title('P_{x}-P_{y}');
grid on
axis equal

%% the position of biped robot 
figure;
plot(tr,F11.xr(1,:));
title('t-P_{x}');
legend('Desired');
xlabel('Time (s)');
ylabel('X-axis (m)');
grid on

figure;
plot(tr,F11.xr(2,:));
title('t-P_{y}');
legend('Desired');
xlabel('Time (s)');
ylabel('Y-axis (m)');
grid on

figure;
plot(tr,F11.xr(3,:));
title('t-\phi');
legend('Desired');
xlabel('Time (s)');
ylabel('\phi (rad)');
grid on


% Conclusion: 利用Reference model 去濾掉高頻震盪
% for i = 1:length(t)
% X{i} = [L.r(1,i),L.r(2,i);r(1,i),r(2,i)];
% Len(i) = pdist(X{i});
% end
% dy = Len./2.*sin(pi/6);
% dx = Len./2.*cos(pi/6);
% 
% x = r(1,:)+dx;
% y = r(2,:)+dy;
% 
% vx = [0,diff(x)/hh];
% vy = [0,diff(y)/hh];
% 
% ax = [0,diff(vx)/hh];
% ay = [0,diff(vy)/hh];
% 
% jx = [0,diff(ax)/hh];
% jy = [0,diff(ay)/hh];
% 
% 
% % figure;
% % plot(t,x);
% % figure;
% % plot(t,y);
% % figure;
% % plot(t,vx);
% % figure;
% % plot(t,vy);
% % figure;
% % plot(t,ax);
% % figure;
% % plot(t,ay);
% % figure;
% % plot(t,jx);
% % figure;
% % plot(t,jy);
% 
% phi = atan2(vy,vx);
% phi(1:16000) = phi(16000)*ones(1,16000);
% phi(end-15999:end) = phi(end-15999)*ones(1,16000);
% w = [0,diff(phi)/hh];
% afa = [0,diff(w)/hh];
% 
% figure;
% plot(t,phi);
% figure;
% plot(t,w);
% figure;
% plot(t,afa);
% 
% R = [phi;w;afa];
% beta = 10;
% AR = -beta*eye(3);
% BR = beta*eye(3);
% 
% xR(:,1) = R(:,1);
% for i = 1:length(tr)-1
%     k1 = com0_RungeKutta(AR,BR,xR(:,i),R(:,2*i-1));
%     k2 = com0_RungeKutta(AR,BR,xR(:,i)+h*k1/2,R(:,2*i));
%     k3 = com0_RungeKutta(AR,BR,xR(:,i)+h*k2/2,R(:,2*i));
%     k4 = com0_RungeKutta(AR,BR,xR(:,i)+h*k3,R(:,2*i+1));
%     xR(:,i+1) = xR(:,i) + 1/6*h*(k1-k2-k3+k4);
%     disp(i);
% end
% 
% for i = 1:3
% figure;
% plot(tr,xR(i,:));
% end
