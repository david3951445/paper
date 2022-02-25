clc;clear;close all
load com0_trajectory.mat
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 從 vy 去求 py時，是否該用Runge-Kutta? 
% Ans: 似乎不太需要，作圖看起來感覺差不多
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% set parameter
nv = 12; % number of variable
hr = hh*2; % real sampling time 
tr = 0:hr:t_end;
h = hr;

%% define reference input
%%% trailer %%%
phir = p(:,1)';wr = v(:,1)';afar = a(:,1)';
pxr = p(:,2)';vxr = v(:,2)';axr = a(:,2)';
% initialization
vyr = vxr.*tan(phir);
pyr = ones(1,length(p))+0.2*sin(30*pi/180);

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
ar = diff(vxr)/hh;
ar = [0 ar];

r = [pxr;pyr;vxr;vyr;axr;ayr;phir;wr;afar;rr;vr;ar];

test_phi = atan(vyr./vxr); % 在剛開始和最後會出現 NaN

figure;
plot(t,phir,t,test_phi)