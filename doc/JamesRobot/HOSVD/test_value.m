clc;clear;
% >> a = 5  % a constant
% a =
%      5
% >> b = 3  % another constant
% b =
%      3
% >> h = @(x) a*x+b  % an anonymous function handle for the linear expression a*x+b
% h = 
%     @(x)a*x+b
% >> h(4)  % evaluate the function handle h for an input of 4
% ans =
%     23
% >> h(7)  % evaluate the function handle h for an input of 7
% ans =
%     38
S = load('G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\HOSVD\leg_trajectory_interpolation_sample.mat');
addpath 'G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot'

h = 1e-4;
th = 15000;
v = 0.1;
% disp(-ff(0+2*h,th,S))
% disp(8*ff(0+h,th,S))
a1 = -ff(v+2*h,th,S);
a2 = 8*ff(v+h,th,S);
a3 = -8*ff(v-h,th,S);
a4 = ff(v-2*h,th,S);
A = (a1+a2+a3+a4)/(12*h);
% gait = (-ff(0+2*h,th,S)+8*ff(0+h,th,S)-8*ff(0-h,th,S)+ff(0-2*h,th,S)) / 12*h;

function gait = ff(v,index,S)
if index>S.len
    index = index-S.len;
end
H = zeros(1,S.gridsize);

for j = 1:S.gridsize
    H(j) = trapmf(v,S.mem(j,:));
end

% 空間腳速度用內插
xq_r = H*S.sample_xq_r_p(:,index);
zq_r = H*S.sample_zq_r_p(:,index);
vx_r = H*S.sample_vx_r_b(:,index);
vz_r = H*S.sample_vz_r_b(:,index);
ax_r = H*S.sample_ax_r_b(:,index);
az_r = H*S.sample_az_r_b(:,index);

xq_l = H*S.sample_xq_l_p(:,index);
zq_l = H*S.sample_zq_l_p(:,index);
vx_l = H*S.sample_vx_l_b(:,index);
vz_l = H*S.sample_vz_l_b(:,index);
ax_l = H*S.sample_ax_l_b(:,index);
az_l = H*S.sample_az_l_b(:,index);

% joint space 的直接算
Tm_r_p = [[eye(3); 0 0 0] [xq_r;0;zq_r;1]];
Tm_r_0 = S.Tm_0*Tm_r_p;
qr = leg_IK(Tm_r_0,S.L3,S.L4,S.L5);
q2 = qr(1); q4 = qr(2); q6 = qr(3); q8 = qr(4);
Tm_l_p = [[eye(3); 0 0 0] [xq_l;0;zq_l;1]];
Tm_l_0 = S.Tm_0*Tm_l_p;
ql = leg_IK(Tm_l_0,S.L3,S.L4,S.L5);
q1 = ql(1); q3 = ql(2); q5 = ql(3); q7 = ql(4);

q = [q1;q2;q3;q4;q5;q6;q7;q8]; % 13~20

dq = leg_Jb_inv(q,S.L4,S.L5)*[vx_r;0;vz_r;0;vx_l;0;vz_l;0]; % 21~28

ddq = leg_Jb_inv(q,S.L4,S.L5)*[ax_r;0;az_r;0;ax_l;0;az_l;0]...
    + leg_dinvJbdt(dq,q,S.L4,S.L5)*[vx_r;0;vz_r;0;vx_l;0;vz_l;0]; % 29~36
% 12 8 8 8 
gait =  [xq_r;vx_r;ax_r;zq_r;vz_r;az_r;xq_l;vx_l;ax_l;zq_l;vz_l;...
    az_l;q;dq;ddq];
end