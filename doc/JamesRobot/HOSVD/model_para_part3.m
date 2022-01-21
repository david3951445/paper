clc;clear;
close all
S = load('G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\HOSVD\leg_trajectory_interpolation_sample.mat');
%% x = [phi0 phi1 w0 w1 phiBR vBR th vxr vyr phi1r wr];4 8 8 2 1 4
%  x = [1    2    3  4  5     6   7  8   9   10    11];
% lpv = {...
%     @(x)Ar(x)       @(x)zeros(12,4) @(x)zeros(12,2) @(x)zeros(12)   @(x)zeros(12,3) @(x)zeros(12)  ;
%     @(x)S3*Ar(x)    @(x)zeros(4)    @(x)Jt(x)       @(x)zeros(4,12) @(x)zeros(4,3)  @(x)zeros(4,12);
%     @(x)J_bar(x)    @(x)zeros(2,4)  @(x)zeros(2,2)  @(x)zeros(2,12) @(x)zeros(2,3)  @(x)zeros(2,12);
%     @(x)zeros(12)   @(x)zeros(12,4) @(x)zeros(12,2) @(x)zeros(12)   @(x)zeros(12,3) @(x)S4*Je(x)   ;
%     @(x)zeros(3,12) @(x)zeros(3,4)  @(x)-S5*Jt(x)   @(x)zeros(3,12) @(x)zeros(3)    @(x)G(x)*S1    ;
%     @(x)G_bar(x)    @(x)zeros(12,4) @(x)zeros(12,2) @(x)zeros(12,12)@(x)zeros(12,3) @(x)zeros(12,12);
% };
%% parameter
S1 = [eye(8) zeros(8,4)];
S2 = [zeros(1,2) eye(1)];
S3 = [eye(4) zeros(4,8)];
S4 = [S1' [zeros(4,2) eye(4) [zeros(2) eye(2)]' eye(4)]']';
S5 = [[eye(2) zeros(2)]' [zeros(1,3) eye(1)]']';
S7 = [S5 zeros(3,2)];
S6 = [zeros(6,12) S7' zeros(6,12)]';
S9 = [zeros(4,2) -eye(4) [zeros(2) -eye(2)]']';
S8 = blkdiag(eye(8),S9)';

d = 20;
beta=2000;
Ar=-beta*eye(12);
%% lpv{4,6}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 利用數值微分去求偏微分 --> 五點法
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Je1_1 = @(x) 0;
Je1_2 = @(x) 0;
Je1_3 = @(x) 0;
Je1_4 = @(x) 1;
Je1_5 = @(x) 0;
Je1_6 = @(x) 0;
Je1_7 = @(x) 0;
Je1_8 = @(x) 0;
Je1_9 = @(x) 0;
Je1_10 = @(x) 0;
Je1_11 = @(x) 0;
Je1_12 = @(x) 0;
Je2_1 = @(x) 0;
Je2_2 = @(x) 0;
Je2_3 = @(x) 0;
Je2_4 = @(x) 1;
Je2_5 = @(x) 0;
Je2_6 = @(x) 0;
Je2_7 = @(x) 0;
Je2_8 = @(x) 0;
Je2_9 = @(x) 0;
Je2_10 = @(x) 0;
Je2_11 = @(x) 0;
Je2_12 = @(x) 0;
Je3_1 = @(x) dq3(x(1),x(2),S);
Je3_2 = @(x) 0;
Je3_3 = @(x) 0;
Je3_4 = @(x) 0;
Je3_5 = @(x) 0;
Je3_6 = @(x) 0;
Je3_7 = @(x) 0;
Je3_8 = @(x) 0;
Je3_9 = @(x) 0;
Je3_10 = @(x) 0;
Je3_11 = @(x) 0;
Je3_12 = @(x) 0;
Je4_1 = @(x) dq4(x(1),x(2),S);
Je4_2 = @(x) 0;
Je4_3 = @(x) 0;
Je4_4 = @(x) 0;
Je4_5 = @(x) 0;
Je4_6 = @(x) 0;
Je4_7 = @(x) 0;
Je4_8 = @(x) 0;
Je4_9 = @(x) 0;
Je4_10 = @(x) 0;
Je4_11 = @(x) 0;
Je4_12 = @(x) 0;
Je5_1 = @(x) dq5(x(1),x(2),S);
Je5_2 = @(x) 0;
Je5_3 = @(x) 0;
Je5_4 = @(x) 0;
Je5_5 = @(x) 0;
Je5_6 = @(x) 0;
Je5_7 = @(x) 0;
Je5_8 = @(x) 0;
Je5_9 = @(x) 0;
Je5_10 = @(x) 0;
Je5_11 = @(x) 0;
Je5_12 = @(x) 0;
Je6_1 = @(x) dq6(x(1),x(2),S);
Je6_2 = @(x) 0;
Je6_3 = @(x) 0;
Je6_4 = @(x) 0;
Je6_5 = @(x) 0;
Je6_6 = @(x) 0;
Je6_7 = @(x) 0;
Je6_8 = @(x) 0;
Je6_9 = @(x) 0;
Je6_10 = @(x) 0;
Je6_11 = @(x) 0;
Je6_12 = @(x) 0;
Je7_1 = @(x) dq7(x(1),x(2),S);
Je7_2 = @(x) 0;
Je7_3 = @(x) 0;
Je7_4 = @(x) 0;
Je7_5 = @(x) 0;
Je7_6 = @(x) 0;
Je7_7 = @(x) 0;
Je7_8 = @(x) 0;
Je7_9 = @(x) 0;
Je7_10 = @(x) 0;
Je7_11 = @(x) 0;
Je7_12 = @(x) 0;
Je8_1 = @(x) dq8(x(1),x(2),S);
Je8_2 = @(x) 0;
Je8_3 = @(x) 0;
Je8_4 = @(x) 0;
Je8_5 = @(x) 0;
Je8_6 = @(x) 0;
Je8_7 = @(x) 0;
Je8_8 = @(x) 0;
Je8_9 = @(x) 0;
Je8_10 = @(x) 0;
Je8_11 = @(x) 0;
Je8_12 = @(x) 0;
Je9_1 = @(x) 0;
Je9_2 = @(x) 0;
Je9_3 = @(x) 0;
Je9_4 = @(x) 0;
Je9_5 = @(x) 0;
Je9_6 = @(x) 0;
Je9_7 = @(x) 0;
Je9_8 = @(x) 0;
Je9_9 = @(x) 1;
Je9_10 = @(x) 0;
Je9_11 = @(x) 0;
Je9_12 = @(x) 0;
Je10_1 = @(x) 0;
Je10_2 = @(x) 0;
Je10_3 = @(x) 0;
Je10_4 = @(x) 0;
Je10_5 = @(x) 0;
Je10_6 = @(x) 0;
Je10_7 = @(x) 0;
Je10_8 = @(x) 0;
Je10_9 = @(x) 0;
Je10_10 = @(x) 1;
Je10_11 = @(x) 0;
Je10_12 = @(x) 0;
Je11_1 = @(x) 0;
Je11_2 = @(x) 0;
Je11_3 = @(x) 0;
Je11_4 = @(x) 0;
Je11_5 = @(x) 0;
Je11_6 = @(x) 0;
Je11_7 = @(x) 0;
Je11_8 = @(x) 0;
Je11_9 = @(x) 0;
Je11_10 = @(x) 0;
Je11_11 = @(x) 1;
Je11_12 = @(x) 0;
Je12_1 = @(x) 0;
Je12_2 = @(x) 0;
Je12_3 = @(x) 0;
Je12_4 = @(x) 0;
Je12_5 = @(x) 0;
Je12_6 = @(x) 0;
Je12_7 = @(x) 0;
Je12_8 = @(x) 0;
Je12_9 = @(x) 0;
Je12_10 = @(x) 0;
Je12_11 = @(x) 0;
Je12_12 = @(x) 1;
%% lpv{6,1} v
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% lpv{6,1} is obtained from idea trajectory Xd(t)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Gbar1_1 = @(x) 0;
Gbar2_1 = @(x) 0;
Gbar3_1 = @(x) 0;
Gbar4_1 = @(x) 0;
Gbar5_1 = @(x) 0;
Gbar6_1 = @(x) 0;
Gbar7_1 = @(x) 0;
Gbar8_1 = @(x) 0;

Gbar1_2 = @(x) 0;
Gbar2_2 = @(x) 0;
Gbar3_2 = @(x) 0;
Gbar4_2 = @(x) 0;
Gbar5_2 = @(x) 0;
Gbar6_2 = @(x) 0;
Gbar7_2 = @(x) 0;
Gbar8_2 = @(x) 0;

Gbar1_3 = @(x) 0;
Gbar2_3 = @(x) 0;
Gbar3_3 = @(x) 0;
Gbar4_3 = @(x) 0;
Gbar5_3 = @(x) 0;
Gbar6_3 = @(x) 0;
Gbar7_3 = @(x) 0;
Gbar8_3 = @(x) 0;

Gbar1_4 = @(x) 0;
Gbar2_4 = @(x) 0;
Gbar3_4 = @(x) 0;
Gbar4_4 = @(x) 0;
Gbar5_4 = @(x) 0;
Gbar6_4 = @(x) 0;
Gbar7_4 = @(x) 0;
Gbar8_4 = @(x) 0;

Gbar1_5 = @(x) -x(6)*sin(x(5));
Gbar2_5 = @(x) x(6)*cos(x(5));
Gbar3_5 = @(x) 0;
Gbar4_5 = @(x) 0;
Gbar5_5 = @(x) 0;
Gbar6_5 = @(x) 0;
Gbar7_5 = @(x) 0;
Gbar8_5 = @(x) 0;

Gbar1_6 = @(x) x(6)*cos(x(5));
Gbar2_6 = @(x) x(6)*sin(x(5));
Gbar3_6 = @(x) 0;
Gbar4_6 = @(x) 0;
Gbar5_6 = @(x) 0;
Gbar6_6 = @(x) 0;
Gbar7_6 = @(x) 0;
Gbar8_6 = @(x) 0;

Gbar1_7 = @(x) 0;
Gbar2_7 = @(x) 0;
Gbar3_7 = @(x) 0;
Gbar4_7 = @(x) 0;
Gbar5_7 = @(x) 0;
Gbar6_7 = @(x) 0;
Gbar7_7 = @(x) 0;
Gbar8_7 = @(x) 0;

Gbar1_8 = @(x) 0;
Gbar2_8 = @(x) 0;
Gbar3_8 = @(x) 0;
Gbar4_8 = @(x) 0;
Gbar5_8 = @(x) 0;
Gbar6_8 = @(x) 0;
Gbar7_8 = @(x) 0;
Gbar8_8 = @(x) 0;

Gbar1_9 = @(x) cos(x(5));
Gbar2_9 = @(x) sin(x(5));
Gbar3_9 = @(x) 0;
Gbar4_9 = @(x) 0;
Gbar5_9 = @(x) 0;
Gbar6_9 = @(x) 0;
Gbar7_9 = @(x) 0;
Gbar8_9 = @(x) 0;

Gbar1_10 = @(x) sin(x(5));
Gbar2_10 = @(x) -cos(x(5));
Gbar3_10 = @(x) 0;
Gbar4_10 = @(x) 0;
Gbar5_10 = @(x) 0;
Gbar6_10 = @(x) 0;
Gbar7_10 = @(x) 0;
Gbar8_10 = @(x) 0;

Gbar1_11 = @(x) 0;
Gbar2_11 = @(x) 0;
Gbar3_11 = @(x) 0;
Gbar4_11 = @(x) 0;
Gbar5_11 = @(x) 0;
Gbar6_11 = @(x) 0;
Gbar7_11 = @(x) 0;
Gbar8_11 = @(x) 0;

Gbar1_12 = @(x) 0;
Gbar2_12 = @(x) 0;
Gbar3_12 = @(x) 0;
Gbar4_12 = @(x) 1;
Gbar5_12 = @(x) 0;
Gbar6_12 = @(x) 0;
Gbar7_12 = @(x) 0;
Gbar8_12 = @(x) 0;

%% lpv{6,1} ddq
Gbar9_1 = @(x) 0;
Gbar10_1 = @(x) 0;
Gbar11_1 = @(x) 0;
Gbar12_1 = @(x) 0;

Gbar9_2 = @(x) 0;
Gbar10_2 = @(x) 0;
Gbar11_2 = @(x) 0;
Gbar12_2 = @(x) 0;

Gbar9_3 = @(x) 0;
Gbar10_3 = @(x) 0;
Gbar11_3 = @(x) 0;
Gbar12_3 = @(x) 0;

Gbar9_4 = @(x) 0;
Gbar10_4 = @(x) 0;
Gbar11_4 = @(x) 0;
Gbar12_4 = @(x) 0;

Gbar9_5 = @(x) ddq9x(x(3),x(4),x(2),S);
Gbar10_5 = @(x) ddq10x(x(3),x(4),x(2),S);
Gbar11_5 = @(x) ddq11x(x(3),x(4),x(2),S);
Gbar12_5 = @(x) ddq12x(x(3),x(4),x(2),S);

Gbar9_6 = @(x) ddq9y(x(3),x(4),x(2),S);
Gbar10_6 = @(x) ddq10y(x(3),x(4),x(2),S);
Gbar11_6 = @(x) ddq11y(x(3),x(4),x(2),S);
Gbar12_6 = @(x) ddq12y(x(3),x(4),x(2),S);

Gbar9_7 = @(x) 0;
Gbar10_7 = @(x) 0;
Gbar11_7 = @(x) 0;
Gbar12_7 = @(x) 0;

Gbar9_8 = @(x) 0;
Gbar10_8 = @(x) 0;
Gbar11_8 = @(x) 0;
Gbar12_8 = @(x) 0;

Gbar9_9 = @(x) 0;
Gbar10_9 = @(x) 0;
Gbar11_9 = @(x) 0;
Gbar12_9 = @(x) 0;

Gbar9_10 = @(x) 0;
Gbar10_10 = @(x) 0;
Gbar11_10 = @(x) 0;
Gbar12_10 = @(x) 0;

Gbar9_11 = @(x) 0;
Gbar10_11 = @(x) 0;
Gbar11_11 = @(x) 0;
Gbar12_11 = @(x) 0;

Gbar9_12 = @(x) 0;
Gbar10_12 = @(x) 0;
Gbar11_12 = @(x) 0;
Gbar12_12 = @(x) 0;


save('model_para_part3.mat');

function dq3 = dq3(x,th,S)
h = 1e-3;
v = x;
df = (-f(v+2*h,th,S)+8*f(v+h,th,S)-8*f(v-h,th,S)+f(v-2*h,th,S)) / (12*h);
dq3 = df(15);
fprintf('dq3: %f,%g\n',dq3,th);
end
function dq4 = dq4(x,th,S)
h = 1e-3;
v = x;
df = (-f(v+2*h,th,S)+8*f(v+h,th,S)-8*f(v-h,th,S)+f(v-2*h,th,S)) / (12*h);
dq4 = df(16);
fprintf('dq4 %g\n',th);
end
function dq5 = dq5(x,th,S)
h = 1e-3;
v = x;
df = (-f(v+2*h,th,S)+8*f(v+h,th,S)-8*f(v-h,th,S)+f(v-2*h,th,S)) / (12*h);
dq5 = df(17);
fprintf('dq5 %g\n',th);
end
function dq6 = dq6(x,th,S)
h = 1e-3;
v = x;
df = (-f(v+2*h,th,S)+8*f(v+h,th,S)-8*f(v-h,th,S)+f(v-2*h,th,S)) / (12*h);
dq6 = df(18);
fprintf('dq6 %g\n',th);
end
function dq7 = dq7(x,th,S)
h = 1e-3;
v = x;
df = (-f(v+2*h,th,S)+8*f(v+h,th,S)-8*f(v-h,th,S)+f(v-2*h,th,S)) / (12*h);
dq7 = df(19);
fprintf('dq7 %g\n',th);
end
function dq8 = dq8(x,th,S)
h = 1e-3;
v = x;
df = (-f(v+2*h,th,S)+8*f(v+h,th,S)-8*f(v-h,th,S)+f(v-2*h,th,S)) / (12*h);
dq8 = df(20);
fprintf('dq8 %g\n',th);
end

function ddq9 = ddq9x(vx,vy,th,S)
h = 1e-3;
vx1 = vx+2*h;
vx2 = vx+h;
vx3 = vx-h;
vx4 = vx-2*h;

v1 = vx1^2+vy^2;
v2 = vx2^2+vy^2;
v3 = vx3^2+vy^2;
v4 = vx4^2+vy^2;

df = (-f(v1,th,S)+8*f(v2,th,S)-8*f(v3,th,S)+f(v4,th,S)) / (12*h);
ddq9 = -df(31);
fprintf('ddq9 %g\n',th);
end

function ddq10 = ddq10x(vx,vy,th,S)
h = 1e-3;
vx1 = vx+2*h;
vx2 = vx+h;
vx3 = vx-h;
vx4 = vx-2*h;

v1 = vx1^2+vy^2;
v2 = vx2^2+vy^2;
v3 = vx3^2+vy^2;
v4 = vx4^2+vy^2;

df = (-f(v1,th,S)+8*f(v2,th,S)-8*f(v3,th,S)+f(v4,th,S)) / (12*h);
ddq10 = -df(32);
fprintf('ddq10 %g\n',th);
end

function ddq11 = ddq11x(vx,vy,th,S)
h = 1e-3;
vx1 = vx+2*h;
vx2 = vx+h;
vx3 = vx-h;
vx4 = vx-2*h;

v1 = vx1^2+vy^2;
v2 = vx2^2+vy^2;
v3 = vx3^2+vy^2;
v4 = vx4^2+vy^2;

df = (-f(v1,th,S)+8*f(v2,th,S)-8*f(v3,th,S)+f(v4,th,S)) / (12*h);
ddq11 = -df(33)-df(35);
fprintf('ddq11 %g\n',th);
end

function ddq12 = ddq12x(vx,vy,th,S)
h = 1e-3;
vx1 = vx+2*h;
vx2 = vx+h;
vx3 = vx-h;
vx4 = vx-2*h;

v1 = vx1^2+vy^2;
v2 = vx2^2+vy^2;
v3 = vx3^2+vy^2;
v4 = vx4^2+vy^2;

df = (-f(v1,th,S)+8*f(v2,th,S)-8*f(v3,th,S)+f(v4,th,S)) / (12*h);
ddq12 = -df(34)-df(36);
fprintf('ddq12 %g\n',th);
end

function ddq9 = ddq9y(vx,vy,th,S)
h = 1e-3;
vy1 = vy+2*h;
vy2 = vy+h;
vy3 = vy-h;
vy4 = vy-2*h;

v1 = vy1^2+vx^2;
v2 = vy2^2+vx^2;
v3 = vy3^2+vx^2;
v4 = vy4^2+vx^2;

df = (-f(v1,th,S)+8*f(v2,th,S)-8*f(v3,th,S)+f(v4,th,S)) / (12*h);
ddq9 = -df(31);
fprintf('ddq9 %g\n',th);
end

function ddq10 = ddq10y(vx,vy,th,S)
h = 1e-3;
vy1 = vy+2*h;
vy2 = vy+h;
vy3 = vy-h;
vy4 = vy-2*h;

v1 = vy1^2+vx^2;
v2 = vy2^2+vx^2;
v3 = vy3^2+vx^2;
v4 = vy4^2+vx^2;

df = (-f(v1,th,S)+8*f(v2,th,S)-8*f(v3,th,S)+f(v4,th,S)) / (12*h);
ddq10 = -df(32);
fprintf('ddq10 %g\n',th);
end

function ddq11 = ddq11y(vx,vy,th,S)
h = 1e-3;
vy1 = vy+2*h;
vy2 = vy+h;
vy3 = vy-h;
vy4 = vy-2*h;

v1 = vy1^2+vx^2;
v2 = vy2^2+vx^2;
v3 = vy3^2+vx^2;
v4 = vy4^2+vx^2;

df = (-f(v1,th,S)+8*f(v2,th,S)-8*f(v3,th,S)+f(v4,th,S)) / (12*h);
ddq11 = -df(33)-df(35);
fprintf('ddq11 %g\n',th);
end

function ddq12 = ddq12y(vx,vy,th,S)
h = 1e-3;
vy1 = vy+2*h;
vy2 = vy+h;
vy3 = vy-h;
vy4 = vy-2*h;

v1 = vy1^2+vx^2;
v2 = vy2^2+vx^2;
v3 = vy3^2+vx^2;
v4 = vy4^2+vx^2;

df = (-f(v1,th,S)+8*f(v2,th,S)-8*f(v3,th,S)+f(v4,th,S)) / (12*h);
ddq12 = -df(34)-df(36);
fprintf('ddq12 %g\n',th);
end

function gait = f(v,index,S)
%% defuzzy
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Problem: v 的點會多兩個
% Solution: 在 xr 多設計兩個點
% --> 繪圖時我們真正要的: length(v)-2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if index > S.len
    index = index - S.len;
end
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

q = [q1;q2;q3;q4;q5;q6;q7;q8]; % 13~20

% redundant joints 


dq = leg_Jb_inv(q,S.L4,S.L5)*[vx_r;0;vz_r;0;vx_l;0;vz_l;0]; % 21~28

ddq = leg_Jb_inv(q,S.L4,S.L5)*[ax_r;0;az_r;0;ax_l;0;az_l;0]...
    + leg_dinvJbdt(dq,q,S.L4,S.L5)*[vx_r;0;vz_r;0;vx_l;0;vz_l;0]; % 29~36

gait =  [xq_r;vx_r;ax_r;zq_r;vz_r;az_r;xq_l;vx_l;ax_l;zq_l;vz_l;...
    az_l;q;dq;ddq];
end
