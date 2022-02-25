clc;clear;close all
load 'com0_trajectory_f1_v1.mat'
%% set parameter
nv = 24; % number of variable
hr = hh*2; % real sampling time 
tr = 0:hr:t_end;
h = hr;

%% load follower data

phif = p(:,1)';wf = v(:,1)';afaf = a(:,1)';
pxf = p(:,2)';vxf = v(:,2)';axf = a(:,2)';
% initialization
vyf = vxf.*tan(phif);
pyf = ones(1,length(p));

for i = 1:length(p)-1
    pyf(i+1) = pyf(i) + vyf(i+1)*hh;
end

ayf = diff(vyf)/hh;
ayf = [0,ayf];

vf = (vxf.^2 + vyf.^2).^(1/2);
rf = zeros(1,length(t));
for i = 1:length(t)-1
    rf(i+1) = rf(i) + vf(i+1)*hh;
end
af = diff(vf)/hh;
af = [0 af];

%% 利用投影點求 leader 的位置 (since the trajectory of leader is a straight line) 

%%% trailer %%%
pxr = zeros(1,length(p));
pyr = zeros(1,length(p));

for i = 1:length(p)
    a = 1;
    b = -sqrt(3);
    c = -1+3*sqrt(3)/2;
    x0 = pxf(i);
    y0 = pyf(i);
    pxr(i) = x0 - a*(a*x0+b*y0+c)/(a^2+b^2);
    pyr(i) = y0 - b*(a*x0+b*y0+c)/(a^2+b^2);
end
vxr = [0,diff(pxr)/hh];
vyr = [0,diff(pyr)/hh];
axr = [0,diff(vxr)/hh];
ayr = [0,diff(vyr)/hh];
phir = phif + (30*pi/180 - phif);
wr = [0,diff(phir)/hh];
afar = [0,diff(wr)/hh];

vr = (vxr.^2 + vyr.^2).^(1/2);
rr = zeros(1,length(t));
for i = 1:length(t)-1
    rr(i+1) = rr(i) + vr(i+1)*hh;
end
ar = diff(vr)/hh;
ar = [0 ar];


%%% tractor %%%
% parameter
d = 0.2;
% initialization
phi0r = zeros(1,length(t));
x0r = zeros(1,length(t));
y0r = zeros(1,length(t));

phi0r(1) = phir(1);
x0r(1) = pxr(1) + d*cos(phir(1));
y0r(1) = pyr(1) + d*sin(phir(1));
v0r = zeros(1,length(t));
vx0r = zeros(1,length(t));
vy0r = zeros(1,length(t));

for i = 1:length(t)
    if i ~= 1
        v0r(i) = vr(i)/cos(phi0r(i)-phir(i)); % O
        vx0r(i) = v0r(i)*cos(phi0r(i)); % O
        vy0r(i) = v0r(i)*sin(phi0r(i)); % O
        x0r(i) = x0r(i-1) + vx0r(i)*hh; % O
        y0r(i) = y0r(i-1) + vy0r(i)*hh; % O
    end
    if i < length(t)-1
        phi0r(i+1) = phir(i+1) + atan(d*wr(i+1)/vr(i+1)); % O
    end
    if i == length(t)-1
        phi0r(i+1) = phir(i);
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
beta = 10;
Ar = -beta*eye(nv);
Br = beta*eye(nv);

% initialization
xr = zeros(nv,length(tr));
xr(:,1) = r(:,1); % initial position

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

save com0_trajectory_reference_model_l1_v2.mat

