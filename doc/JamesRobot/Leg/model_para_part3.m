clc;clear;close all
S = load('G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\test_Leg\leg_trajectory_interpolation_sample.mat');
%% x(1) = v, x(2) = index(=th)
%% parameter
beta = 10;
Ar = -beta*eye(5);
%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 利用數值微分去求偏微分 --> 五點法
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Je1_1 = @(x) ddq9(x(1),x(2),S);
Je2_1 = @(x) ddq10(x(1),x(2),S);
Je3_1 = @(x) ddq11(x(1),x(2),S);
Je4_1 = @(x) ddq12(x(1),x(2),S);


function ddq9 = ddq9(x,th,S)
h = 1e-4;
v = x;
df = (-f(v+2*h,th,S)+8*f(v+h,th,S)-8*f(v-h,th,S)+f(v-2*h,th,S)) / (12*h);
ddq9 = df(31);
% fprintf('ddq9: %g\n',th);
end
function ddq10 = ddq10(x,th,S)
h = 1e-4;
v = x;
df = (-f(v+2*h,th,S)+8*f(v+h,th,S)-8*f(v-h,th,S)+f(v-2*h,th,S)) / (12*h);
ddq10 = df(32);
% fprintf('ddq10: %g\n',th);
end
function ddq11 = ddq11(x,th,S)
h = 1e-4;
v = x;
df = (-f(v+2*h,th,S)+8*f(v+h,th,S)-8*f(v-h,th,S)+f(v-2*h,th,S)) / (12*h);
ddq11 = df(33)+df(35);
% fprintf('ddq11: %g\n',th);
end
function ddq12 = ddq12(x,th,S)
h = 1e-4;
v = x;
df = (-f(v+2*h,th,S)+8*f(v+h,th,S)-8*f(v-h,th,S)+f(v-2*h,th,S)) / (12*h);
ddq12 = df(34)+df(36);
% fprintf('ddq12: %g\n',th);
end

function gait = f(v,index,S)
tic;
if index > S.len
    index = index - S.len;
end

disp(index);

H = zeros(1,S.gridsize);

for j = 1:S.gridsize
    H(j) = trapmf(v,S.mem(j,:));
end
disp(index);

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

% active joint 
q = [q1;q2;q3;q4;q5;q6;q7;q8]; % 13~20

dq = leg_Jb_inv(q,S.L4,S.L5)*[vx_r;0;vz_r;0;vx_l;0;vz_l;0]; % 21~28

ddq = leg_Jb_inv(q,S.L4,S.L5)*[ax_r;0;az_r;0;ax_l;0;az_l;0]...
    + leg_dinvJbdt(dq,q,S.L4,S.L5)*[vx_r;0;vz_r;0;vx_l;0;vz_l;0]; % 29~36
% 12 
% 8 8 8 
gait =  [...
    xq_r;vx_r;ax_r;zq_r;vz_r;az_r;xq_l;vx_l;ax_l;zq_l;vz_l;az_l;...
    q;dq;ddq
    ];
toc;
end
