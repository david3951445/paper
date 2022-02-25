%% Since the shift distance is fixed, only the position of robot will changed.
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

dy = 0.5/2*sin(pi/6);
dx = 0.5/2*cos(pi/6);

pxl2 = F.r(1,:)+dx;
pyl2 = F.r(2,:)+dy;
vxl2 = F.vxr;
vyl2 = F.vyr;
axl2 = F.axr;
ayl2 = F.ayr;
phil2 = F.phir;
wl2 = F.wr;
afal2 = F.afar;
rl2 = F.rr;
vl2 = F.vr;
al2 = F.ar;

pxl3 = F.r(1,:)-dx;
pyl3 = F.r(2,:)-dy;
vxl3 = F.vxr;
vyl3 = F.vyr;
axl3 = F.axr;
ayl3 = F.ayr;
phil3 = F.phir;
wl3 = F.wr;
afal3 = F.afar;
rl3 = F.rr;
vl3 = F.vr;
al3 = F.ar;


pxu2 = F.r(13,:)+dx;
pyu2 = F.r(14,:)+dy;
vxu2 = F.pjvxr;
vyu2 = F.pjvyr;
axu2 = F.pjaxr;
ayu2 = F.pjayr;
phiu2 = F.pjphir;
wu2 = F.pjwr;
afau2 = F.pjafar;
ru2 = F.pjrr;
vu2 = F.pjvr;
au2 = F.pjar;


pxu3 = F.r(13,:)-dx;
pyu3 = F.r(14,:)-dy;
vxu3 = F.pjvxr;
vyu3 = F.pjvyr;
axu3 = F.pjaxr;
ayu3 = F.pjayr;
phiu3 = F.pjphir;
wu3 = F.pjwr;
afau3 = F.pjafar;
ru3 = F.pjrr;
vu3 = F.pjvr;
au3 = F.pjar;

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
save('com0_trajectory_reference_model_team_fixedlength.mat','xr','r','t','tr','hh','hr');
