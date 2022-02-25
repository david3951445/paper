clc;clear;close all
load com0_trajectory.mat
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 從 vy 去求 py時，是否該用Runge-Kutta? 
% Ans: 似乎不太需要，作圖看起來感覺差不多
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% set parameter
nv = 24; % number of variable
hr = hh*2; % real sampling time 
tr = 0:hr:t_end;
h = hr;

%% define reference input
%%% trailer %%%
phir = p(:,1)';wr = v(:,1)';afar = a(:,1)';
pxr = p(:,2)';vxr = v(:,2)';axr = a(:,2)';
% initialization
vyr = vxr.*tan(phir);
pyr = ones(1,length(p));

for i = 1:length(p)-1
    pyr(i+1) = pyr(i) + vyr(i+1)*hh;
end

ayr = diff(vyr)/hh;
ayr = [0,ayr];

r = [pxr;pyr;vxr;vyr;axr;ayr;phir;wr;afar]; % reference input
vr = (vxr.^2 + vyr.^2).^(1/2);
rr = zeros(1,length(t));
for i = 1:length(t)-1
    rr(i+1) = rr(i) + vr(i+1)*hh;
end
ar = diff(vr)/hh;
ar = [0 ar];
%%% tactor %%%
% parameter
d = 0.2;
% initialization
phi0r = zeros(1,length(t));
x0r = zeros(1,length(t));
y0r = zeros(1,length(t));

phi0r(1) = r(7,1);
x0r(1) = r(1,1) + d*cos(r(7,1));
y0r(1) = r(2,1) + d*sin(r(7,1));
v0r = zeros(1,length(t));
vx0r = zeros(1,length(t));
vy0r = zeros(1,length(t));

for i = 1:length(t)
    if i ~= 1
        v0r(i) = vr(i)/cos(phi0r(i)-r(7,i)); % O
        vx0r(i) = v0r(i)*cos(phi0r(i)); % O
        vy0r(i) = v0r(i)*sin(phi0r(i)); % O
        x0r(i) = x0r(i-1) + vx0r(i)*hh; % O
        y0r(i) = y0r(i-1) + vy0r(i)*hh; % O
    end
    if i < length(t)-1
        phi0r(i+1) = r(7,i+1) + atan(d*r(8,i+1)/vr(i+1)); % O
    end
    if i == length(t)-1
        phi0r(i+1) = r(7,i);
    end
end
r0r = zeros(1,length(t));
for i = 1:length(t)-1
    r0r(i+1) = r0r(i) + v0r(i+1)*hh;
end
a0r = diff(v0r)/hh; a0r = [0 a0r];
ax0r = diff(vx0r)/hh; ax0r = [0 ax0r];
ay0r = diff(vy0r)/hh; ay0r = [0 ay0r];
w0r = diff(phi0r)/hh; w0r = [0 w0r];
afa0r = diff(w0r)/hh; afa0r = [0 afa0r];
r = [pxr;pyr;vxr;vyr;axr;ayr;phir;wr;afar;rr;vr;ar;...
    x0r;y0r;vx0r;vy0r;ax0r;ay0r;phi0r;w0r;afa0r;r0r;v0r;a0r
    ];
%% define reference state
bata = 10;
Ar = -bata*eye(nv);
Br = bata*eye(nv);

% initialization
xr(:,1) = zeros(nv,1);
xr(1:2,1) = [1;1]; % initial position
xr(7,1) = p(1,1);
xr(13,1) = r(1,1) + d*cos(r(7,1));
xr(14,1) = r(2,1) + d*sin(r(7,1));
xr(19,1) = xr(7,1);

for i = 1:length(tr)-1
    k1 = com0_RungeKutta(Ar,Br,xr(:,i),r(:,2*i-1));
    k2 = com0_RungeKutta(Ar,Br,xr(:,i)+h*k1/2,r(:,2*i));
    k3 = com0_RungeKutta(Ar,Br,xr(:,i)+h*k2/2,r(:,2*i));
    k4 = com0_RungeKutta(Ar,Br,xr(:,i)+h*k3,r(:,2*i+1));
    xr(:,i+1) = xr(:,i) + 1/6*h*(k1+2*k2+2*k3+k4);
    disp(i);
end

v = sqrt(xr(3,:).^2+xr(4,:).^2);
v0 = sqrt(xr(15,:).^2+xr(16,:).^2);
%% 使 reference inpput 的 sapmling time 和 xr 一樣
t = 0:hh:t_end;
for i = 1:length(tr)
    r_new(:,i) = r(:,2*i-1);
end

% %% design tractor trajectory
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % design process is in \tractor-trailer\ReferenceTrajectory_v2.m
% % What we really need: phi0, w0, afa0. 
% %       (From reference model state-space definition)
% % xr = [px1r;py1r;vx1r;vy1r;ax1r;ay1r;phi1r;w1r;afa1r;phi0r;w0r;afa0r;];
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % parameter
% d = 0.2; % link bar length: 20cm
% % initialization
% phi0 = zeros(1,length(tr));
% x0 = zeros(1,length(tr));
% y0 = zeros(1,length(tr));
% 
% phi0(1) = xr(7,1);
% x0(1) = xr(1,1) + d*cos(xr(7,1));
% y0(1) = xr(2,1) + d*sin(xr(7,1));
% v0 = zeros(1,length(tr));
% vx0 = zeros(1,length(tr));
% vy0 = zeros(1,length(tr));
% % nonholonomic
% for i = 1:length(tr)
%     if i ~= 1
%         v0(i) = xr(3,i)/cos(phi0(i)-xr(7,i))/cos(xr(7,i)); % O
%         vx0(i) = v0(i)*cos(phi0(i)); % O
%         vy0(i) = v0(i)*sin(phi0(i)); % O
%         x0(i) = x0(i-1) + vx0(i)*hr; % O
%         y0(i) = y0(i-1) + vy0(i)*hr; % O
%     end
%     if i ~= length(tr)
%         phi0(i+1) = xr(7,i+1) + atan(d*xr(8,i+1)/v(i+1)); % O
%     end
% end
% ax0 = diff(vx0)/hr; ax0 = [0 ax0];
% ay0 = diff(vy0)/hr; ay0 = [0 ay0];
% w0 = diff(phi0)/hr; w0 = [0 w0];
% afa0 = diff(w0)/hr; afa0 = [0 afa0];
% 
% xr = [xr;phi0;w0;afa0];

save com0_trajectory_reference_model.mat