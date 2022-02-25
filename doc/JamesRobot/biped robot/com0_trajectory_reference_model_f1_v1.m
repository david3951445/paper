clc;clear;close all
load com0_trajectory_f1_v1.mat
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

vr = (vxr.^2 + vyr.^2).^(1/2);
rr = zeros(1,length(t));
for i = 1:length(t)-1
    rr(i+1) = rr(i) + vr(i+1)*hh;
end
ar = diff(vr)/hh;
ar = [0 ar];

%% 對稱點 
pjpxr = zeros(1,length(p));
pjpyr = zeros(1,length(p));

for i = 1:length(p)
    a = 1;
    b = -sqrt(3);
    c = -1+3*sqrt(3)/2;
    x0 = pxr(i);
    y0 = pyr(i);
    pjpxr(i) = x0 - 2*a*(a*x0+b*y0+c)/(a^2+b^2);
    pjpyr(i) = y0 - 2*b*(a*x0+b*y0+c)/(a^2+b^2);
end
pjvxr = [0,diff(pjpxr)/hh];
pjvyr = [0,diff(pjpyr)/hh];
pjaxr = [0,diff(pjvxr)/hh];
pjayr = [0,diff(pjvyr)/hh];
pjphir = phir + 2*(30*pi/180 - phir);
pjwr = [0,diff(pjphir)/hh];
pjafar = [0,diff(pjwr)/hh];

pjvr = (pjvxr.^2 + pjvyr.^2).^(1/2);
pjrr = zeros(1,length(t));
for i = 1:length(t)-1
    pjrr(i+1) = rr(i) + vr(i+1)*hh;
end
pjar = diff(pjvr)/hh;
pjar = [0 pjar];


%%
r = [pxr;pyr;vxr;vyr;axr;ayr;phir;wr;afar;rr;vr;ar;...
    pjpxr;pjpyr;pjvxr;pjvyr;pjaxr;pjayr;pjphir;pjwr;pjafar;pjrr;pjvr;pjar];
%% define reference state
beta = 10;
Ar = -beta*eye(nv);
Br = beta*eye(nv);

% initialization
xr(:,1) = zeros(nv,1);
xr(1:2,1) = [pxr(1);pyr(1)]; % initial position
xr(7,1) = p(1,1);
xr(13:14,1) = [pjpxr(1);pjpyr(1)]; % initial position
xr(19,1) = pjphir(1);


for i = 1:length(tr)-1
    k1 = com0_RungeKutta(Ar,Br,xr(:,i),r(:,2*i-1));
    k2 = com0_RungeKutta(Ar,Br,xr(:,i)+h*k1/2,r(:,2*i));
    k3 = com0_RungeKutta(Ar,Br,xr(:,i)+h*k2/2,r(:,2*i));
    k4 = com0_RungeKutta(Ar,Br,xr(:,i)+h*k3,r(:,2*i+1));
    xr(:,i+1) = xr(:,i) + 1/6*h*(k1+2*k2+2*k3+k4);
    disp(i);
end

v = sqrt(xr(3,:).^2+xr(4,:).^2);
pjv = sqrt(xr(15,:).^2+xr(16,:).^2);


save com0_trajectory_reference_model_f1_v1.mat

