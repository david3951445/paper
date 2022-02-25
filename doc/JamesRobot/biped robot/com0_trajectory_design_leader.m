function com0_trajectory_design_leader(team)
eval(['F = load("com0_trajectory_reference_model_team' num2str(team) '_follower2.mat");']);
%% set parameter
h = F.h;
hh = F.hh;
tr = F.tr;
t = F.t;
nv = F.nv;

%% 利用投影點求 leader 的位置 (since the trajectory of leader is a straight line) 

%%% trailer %%%
pxr = zeros(1,length(t));
pyr = zeros(1,length(t));

for i = 1:length(t)
    a = 1;
    b = -sqrt(3);
    c = 0;
    x0 = F.r(1,i);
    y0 = F.r(2,i);
    pxr(i) = x0 - a*(a*x0+b*y0+c)/(a^2+b^2);
    pyr(i) = y0 - b*(a*x0+b*y0+c)/(a^2+b^2);
end
vxr = [0,diff(pxr)/hh];
vyr = [0,diff(pyr)/hh];
axr = [0,diff(vxr)/hh];
ayr = [0,diff(vyr)/hh];
phir = F.r(7,:) + (30*pi/180 - F.r(7,:));
wr = zeros(1,length(t));%% since the trajectory is a straight line
afar = zeros(1,length(t));

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

eval(['save("com0_trajectory_reference_model_team' num2str(team) '_leader.mat","xr","r","t","tr","hh","h","nv");']);
end
