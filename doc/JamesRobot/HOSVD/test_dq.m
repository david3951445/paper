%% make sure beta = -10 is negative enough to track dq11,dq12.

clc;clear; close all
load leg_trajectory_interpolation.mat
Gait(1,:) = -gait(17,:)-gait(19,:);
Gait(2,:) = -gait(18,:)-gait(20,:);
Gait(3,:) = -gait(25,:)-gait(27,:);
Gait(4,:) = -gait(26,:)-gait(28,:);
Gait(5,:) = -gait(33,:)-gait(35,:);
Gait(6,:) = -gait(34,:)-gait(36,:);


Xr(:,1) = [Gait(1,1);Gait(2,1);Gait(3,1);Gait(4,1);Gait(5,1);Gait(6,1)];

Ar = -10*eye(6);
Br = 10*eye(6);
for i = 1:length(tr)-1
    k1 = com0_RungeKutta(Ar,Br,Xr(:,i),Gait(:,2*i-1));
    k2 = com0_RungeKutta(Ar,Br,Xr(:,i)+h*k1/2,Gait(:,2*i));
    k3 = com0_RungeKutta(Ar,Br,Xr(:,i)+h*k2/2,Gait(:,2*i));
    k4 = com0_RungeKutta(Ar,Br,Xr(:,i)+h*k3,Gait(:,2*i+1));
    Xr(:,i+1) = Xr(:,i) + 1/6*h*(k1+2*k2+2*k3+k4);
    disp(i);
end
%%
figure;
plot(t,-gait(17,:)-gait(19,:),tr,Xr(1,:));
figure;
plot(t,-gait(18,:)-gait(20,:),tr,Xr(2,:));
figure;
plot(t,-gait(25,:)-gait(27,:),tr,Xr(3,:));
figure;
plot(t,-gait(26,:)-gait(28,:),tr,Xr(4,:));
figure;
plot(t,-gait(33,:)-gait(35,:),tr,Xr(5,:));
figure;
plot(t,-gait(34,:)-gait(36,:),tr,Xr(6,:));






function kr = com0_RungeKutta(A,B,x,r)
kr = A*x + B*r;
end