clc;clear;close all
%%% generate gait with fixed velocity %%% 
v = 2;
%%% by cubic spline %%%

%% p
c = 0.2;
S = c*v; % one step gait
th = [pi pi/2 0 -pi/2 -pi] ;
x = [-S 0 S 0 -S];
h = 1e4;
thq = linspace(pi,-pi,h);
thq_shifted = -thq; %% since th is from pi to -pi
xq = spline(th,x,thq);
figure; 
plot(thq_shifted,xq);

% thz1 = [pi pi/2 0];
hf = 1;
% z1 = [0 hf 0];
% thz1q = linspace(pi,0,0.5*h);
% z1q = spline(thz1,z1,thz1q);
% thz2 = [0 -pi/2 -pi];
% z2 = [0 0 0];
% thz2q = linspace(0,-pi,0.5*h);
% z2q = spline(thz2,z2,thz2q);
% thzq = [thz1q,thz2q];
% zq = [z1q,z2q];
% thzq_shifted = -thzq;
for i = 1:length(thq)
    if thq(i) >= 0
        zq(i) = hf/2*(1+sin(2*thq(i)-pi/2));
    else
        zq(i) = 0;
    end
end



figure; 
plot(thq_shifted,zq);
figure;
plot(xq,zq);
comet(xq,zq);

%% v
hh = thq(1)-thq(2);
vx = diff(xq)/hh;
vx = [vx vx(1)]; %% since it is period function 
figure; 
plot(thq_shifted,vx);
vz = diff(zq)/hh;
vz = [vz vz(1)];
figure; 
plot(thq_shifted,vz);

%% acc
ax = diff(vx)/hh;
ax = [ax ax(1)];
figure; 
plot(thq_shifted,ax);
az = diff(vz)/hh;
az = [az az(1)];
figure; 
plot(thq_shifted,az);
