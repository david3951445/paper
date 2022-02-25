% Design other followers (4)
clc;clear;close all
F = load('com0_trajectory_reference_model_f1_v1.mat');
L = load('com0_trajectory_reference_model_l1_v2.mat');

t = F.t;
tr = F.tr;
hh = F.hh;
hr = F.hr;

pxl1 = F.pxr;
pyl1 = F.pyr;
vxl1 = F.vxr;
vyl1 = F.vyr;
axl1 = F.axr;
ayl1 = F.ayr;
phil1 = F.phir;
wl1 = F.wr;
afal1 = F.afar;
rl1 = F.rr;
vl1 = F.vr;
al1 = F.ar;

pxu1 = F.pjpxr;
pyu1 = F.pjpyr;
vxu1 = F.pjvxr;
vyu1 = F.pjvyr;
axu1 = F.pjaxr;
ayu1 = F.pjayr;
phiu1 = F.pjphir;
wu1 = F.pjwr;
afau1 = F.pjafar;
ru1 = F.pjrr;
vu1 = F.pjvr;
au1 = F.pjar;

X = cell(1,length(t));
Dis = zeros(1,length(t));
for i = 1:length(t)
X{i} = [L.r(1,i),L.r(2,i);F.r(1,i),F.r(2,i)];
Dis(i) = pdist(X{i});
end
dy = Dis./2.*sin(pi/6);
dx = Dis./2.*cos(pi/6);


pxl2 = F.r(1,:)+dx;
pyl2 = F.r(2,:)+dy;
vxl2 = [0,diff(pxl2)/hh];
vyl2 = [0,diff(pyl2)/hh];
axl2 = [0,diff(vxl2)/hh];
ayl2 = [0,diff(vyl2)/hh];
vl2 = (vxl2.^2 + vyl2.^2).^(1/2);
rl2 = zeros(1,length(t));
for i = 1:length(t)-1
    rl2(i+1) = rl2(i) + vl2(i+1)*hh;
end
al2 = [0,diff(vl2)/hh;];

pxl3 = F.r(1,:)-dx;
pyl3 = F.r(2,:)-dy;
vxl3 = [0,diff(pxl3)/hh];
vyl3 = [0,diff(pyl3)/hh];
axl3 = [0,diff(vxl3)/hh];
ayl3 = [0,diff(vyl3)/hh];
vl3 = (vxl3.^2 + vyl3.^2).^(1/2);
rl3 = zeros(1,length(t));
for i = 1:length(t)-1
    rl3(i+1) = rl3(i) + vl3(i+1)*hh;
end
al3 = [0,diff(vl3)/hh;];

pxu2 = F.r(13,:)+dx;
pyu2 = F.r(14,:)+dy;
vxu2 = [0,diff(pxu2)/hh];
vyu2 = [0,diff(pyu2)/hh];
axu2 = [0,diff(vxu2)/hh];
ayu2 = [0,diff(vyu2)/hh];
vu2 = (vxu2.^2 + vyu2.^2).^(1/2);
ru2 = zeros(1,length(t));
for i = 1:length(t)-1
    ru2(i+1) = ru2(i) + vu2(i+1)*hh;
end
au2 = [0,diff(vu2)/hh;];


pxu3 = F.r(13,:)-dx;
pyu3 = F.r(14,:)-dy;
vxu3 = [0,diff(pxu3)/hh];
vyu3 = [0,diff(pyu3)/hh];
axu3 = [0,diff(vxu3)/hh];
ayu3 = [0,diff(vyu3)/hh];
vu3 = (vxu3.^2 + vyu3.^2).^(1/2);
ru3 = zeros(1,length(t));
for i = 1:length(t)-1
    ru3(i+1) = ru3(i) + vu3(i+1)*hh;
end
au3 = [0,diff(vu3)/hh;];


cs = 16000; % changed seconds
phil2 = atan2(vyl2,vxl2);
phil2(1:cs) = phil2(cs)*ones(1,cs);
phil2(end-cs+1:end) = phil2(end-cs)*ones(1,cs);
wl2 = [0,diff(phil2)/hh];
afal2 = [0,diff(wl2)/hh];

phil3 = atan2(vyl3,vxl3);
phil3(1:cs) = phil3(cs)*ones(1,cs);
phil3(end-cs+1:end) = phil3(end-cs)*ones(1,cs);
wl3 = [0,diff(phil3)/hh];
afal3 = [0,diff(wl3)/hh];

phiu2 = atan2(vyu2,vxu2);
phiu2(1:cs) = phiu2(cs)*ones(1,cs);
phiu2(end-cs+1:end) = phiu2(end-cs)*ones(1,cs);
wu2 = [0,diff(phiu2)/hh];
afau2 = [0,diff(wu2)/hh];

phiu3 = atan2(vyu3,vxu3);
phiu3(1:cs) = phiu3(cs)*ones(1,cs);
phiu3(end-cs+1:end) = phiu3(end-cs)*ones(1,cs);
wu3 = [0,diff(phiu3)/hh];
afau3 = [0,diff(wu3)/hh];

%%
r = [pxl1;pyl1;vxl1;vyl1;axl1;ayl1;phil1;wl1;afal1;rl1;vl1;al1;...
    pxl2;pyl2;vxl2;vyl2;axl2;ayl2;phil2;wl2;afal2;rl2;vl2;al2;...
    pxl3;pyl3;vxl3;vyl3;axl3;ayl3;phil3;wl3;afal3;rl3;vl3;al3;...
    pxu1;pyu1;vxu1;vyu1;axu1;ayu1;phiu1;wu1;afau1;ru1;vu1;au1;...
    pxu2;pyu2;vxu2;vyu2;axu2;ayu2;phiu2;wu2;afau2;ru2;vu2;au2;...
    pxu3;pyu3;vxu3;vyu3;axu3;ayu3;phiu3;wu3;afau3;ru3;vu3;au3
    ];

%% define reference state
beta = 10;
nv = 12*6; % number of variable
Ar = -beta*eye(nv);
Br = beta*eye(nv);

% initialization
xr = zeros(nv,length(tr));
xr(:,1) = r(:,1);


for i = 1:length(tr)-1
    k1 = com0_RungeKutta(Ar,Br,xr(:,i),r(:,2*i-1));
    k2 = com0_RungeKutta(Ar,Br,xr(:,i)+hr*k1/2,r(:,2*i));
    k3 = com0_RungeKutta(Ar,Br,xr(:,i)+hr*k2/2,r(:,2*i));
    k4 = com0_RungeKutta(Ar,Br,xr(:,i)+hr*k3,r(:,2*i+1));
    xr(:,i+1) = xr(:,i) + 1/6*hr*(k1+2*k2+2*k3+k4);
    disp(i);
end

%%
save('com0_trajectory_reference_model_team.mat','xr','r','t','tr','hh','hr');
