clc;clear;close all
addpath 'G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot'
load('G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot\com0_trajectory_reference_model_team1_follower1.mat','t_end','hh');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 座標轉換
% inverse Kinematics: the position of leg is relative to frame{0} 
% inverse Jacobian: the velocity of leg is relative to frame{b}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 撒點
domain = [0,0.2];
gridsize = 20;
sample_v = linspace(domain(1),domain(2),gridsize);
sample_v = sample_v';

%% parameter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% when time period of one gait is 2*T 
% 0 ~ 2T is matching to pi ~ -pi
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Tp = 8; % time period of a single step
step = 20;
Tp = t_end/step;
tp = 0:hh:2*Tp-hh; % time instant of a complete step (2 single step) 
len = length(tp); 
hf = 0.04; % the height of gait
c = 0.25; % the constant multiply to v --> deteremine the gait length
dt = hh;

% link length
L1 = 0.035;
L2 = 0.0907;
L3 = 0.0285;
L4 = 0.11;
L5 = 0.11;
L6 = 0.0305;
%% generate right gait (Cartesian) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% trajectory is designed on frame{p} 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
S = c*sample_v; % one step gait
Sx = S*Tp/4; % real one step gait (x)
Sz = hf*Tp/4; % real one step gait (z)
% th_r_p = [pi pi/2 0 -pi/2 -pi] ;
% sample_x_r_p = [-S zeros(gridsize,1) S zeros(gridsize,1) -S];
thq_r_p = linspace(pi,-pi,len);
% % cubic spline
% for i = 1:gridsize
%     sample_xq_r_p(i,:) = spline(th_r_p,sample_x_r_p(i,:),thq_r_p);
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% x: design from velocity
% the velocity in frame{p} is same as the velocity in frame {b}
% Reason: the direction of x-axis is same in frame{p} and frame{b}
% Q: Why do not choose spline?
% A: the velocity will be undifferentiable
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sample_vx_r_b = zeros(gridsize,len);
for i = 1:gridsize
    for j = 1:len
        if thq_r_p(j) >= 0
            sample_vx_r_b(i,j) = S(i)/2*(1+sin(-2*(thq_r_p(j)-3*pi/4)));
        else
            sample_vx_r_b(i,j) = -S(i)/2*(1+sin(-2*(thq_r_p(j)-3*pi/4)));
        end
    end
end

sample_xq_r_p = zeros(gridsize,len);
for i = 1:gridsize
    sample_xq_r_p(i,:) = -Sx(i)*ones(1,len);
end

for i = 1:gridsize
    for j = 1:len-1
        sample_xq_r_p(i,j+1) = sample_xq_r_p(i,j) + sample_vx_r_b(i,j+1)*dt;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% z: design from velocity
% the velocity in frame{p} is same as the velocity in frame {b}
% Reason: the direction of z-axis is same in frame{p} and frame{b}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sample_vz_r_b = zeros(gridsize,len);
for i = 1:gridsize
    for j = 1:length(thq_r_p)
        if thq_r_p(j) >= pi/2
            sample_vz_r_b(i,j) = hf/2*(1+sin(-4*(thq_r_p(j)-7*pi/8)));
        elseif thq_r_p(j) >= 0
            sample_vz_r_b(i,j) = -hf/2*(1+sin(-4*(thq_r_p(j)-7*pi/8)));
        else
            sample_vz_r_b(i,j) = 0;
        end
    end
end

sample_zq_r_p = zeros(gridsize,len);
for i = 1:gridsize
    for j = 1:len-1
        sample_zq_r_p(i,j+1) = sample_zq_r_p(i,j) + sample_vz_r_b(i,j+1)*dt;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 座標轉換: frame{0} to frame{p}
% Reason: 因為之後進行Inverse Kinematic所要求的空間腳位置是要相對於frame{0}
% 座標轉換: frame{b} to frame{p}
% Reason: 因為之後進行Inverse Jacobian所要求的空間腳速度是要相對於frame{b}
% Remark: 這邊不使用Inverse Jacobain，直接使用diff(.) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
eul_0 = [-pi/2 0 0]; 
rotmZYX_0 = eul2rotm(eul_0);
ptrans_0 = [0;0;-0.21];
Tm_0 = [[rotmZYX_0 ptrans_0]; 0 0 0 1];

eul_b = [0 0 0]; 
rotmZYX_b = eul2rotm(eul_b);
ptrans_b_r = [0;-L1;-L2-0.21];
Tm_b_r = [[rotmZYX_b ptrans_b_r]; 0 0 0 1];

sample_trans_r_p = zeros(4,len,gridsize);
sample_trans_r_0 = zeros(4,len,gridsize);
sample_trans_r_b = zeros(4,len,gridsize);
for i = 1:gridsize
    sample_trans_r_p(:,:,i) = [sample_xq_r_p(i,:);zeros(1,len);sample_zq_r_p(i,:);ones(1,len)];
    sample_trans_r_0(:,:,i) = Tm_0*sample_trans_r_p(:,:,i);
    sample_trans_r_b(:,:,i) = Tm_b_r*sample_trans_r_p(:,:,i);
end

%%% vx = dx/dth * dth/dt
sample_ax_r_b = zeros(gridsize,len);
sample_az_r_b = zeros(gridsize,len);

for i = 1:gridsize
%%% a %%%
sample_ax_r_b(i,2:end) = diff(sample_vx_r_b(i,:))/dt;
sample_az_r_b(i,2:end) = diff(sample_vz_r_b(i,:))/dt;
end

%% generate left gait (Cartesian)

% th_l_p = [2*pi 3*pi/2 pi pi/2 0] ;
thq_l_p = linspace(2*pi,0,len);

sample_vx_l_b = zeros(gridsize,len);
for i = 1:gridsize
    for j = 1:len
        if thq_l_p(j) >= pi
            sample_vx_l_b(i,j) = -S(i)/2*(1+sin(-2*(thq_l_p(j)-3*pi/4)));
        else
            sample_vx_l_b(i,j) = S(i)/2*(1+sin(-2*(thq_l_p(j)-3*pi/4)));
        end
    end
end

sample_xq_l_p = zeros(gridsize,len);
for i = 1:gridsize
    sample_xq_l_p(i,:) = Sx(i)*ones(1,len);
end

for i = 1:gridsize
    for j = 1:len-1
        sample_xq_l_p(i,j+1) = sample_xq_l_p(i,j) + sample_vx_l_b(i,j+1)*dt;
    end
end

%%% z %%%
sample_vz_l_b = zeros(gridsize,len);
for i = 1:gridsize
    for j = 1:length(thq_r_p)
        if thq_l_p(j) >= pi
            sample_vz_l_b(i,j) = 0;
        elseif thq_l_p(j) >= pi/2
            sample_vz_l_b(i,j) = hf/2*(1+sin(-4*(thq_l_p(j)-7*pi/8)));
        else
            sample_vz_l_b(i,j) = -hf/2*(1+sin(-4*(thq_l_p(j)-7*pi/8)));
        end
    end
end

sample_zq_l_p = zeros(gridsize,len);
for i = 1:gridsize
    for j = 1:len-1
        sample_zq_l_p(i,j+1) = sample_zq_l_p(i,j) + sample_vz_l_b(i,j+1)*dt;
%         if sample_zq_l_p(i,j+1) < 1e-12
%             sample_zq_l_p(i,j+1) = 0;
%         end
    end
end
ptrans_b_l = [0;L1;-L2-0.21];
Tm_b_l = [[rotmZYX_b ptrans_b_l]; 0 0 0 1];


sample_trans_l_p = zeros(4,len,gridsize);
sample_trans_l_0 = zeros(4,len,gridsize);
sample_trans_l_b = zeros(4,len,gridsize);
for i = 1:gridsize
    sample_trans_l_p(:,:,i) = [sample_xq_l_p(i,:);zeros(1,len);sample_zq_l_p(i,:);ones(1,len)];
    sample_trans_l_0(:,:,i) = Tm_0*sample_trans_l_p(:,:,i);
    sample_trans_l_b(:,:,i) = Tm_b_l*sample_trans_l_p(:,:,i);
end

%%% vx = dx/dth * dth/dt
sample_ax_l_b = zeros(gridsize,len);
sample_az_l_b = zeros(gridsize,len);

for i = 1:gridsize
%%% a %%%
sample_ax_l_b(i,2:end) = diff(sample_vx_l_b(i,:))/dt;
sample_az_l_b(i,2:end) = diff(sample_vz_l_b(i,:))/dt;
end
%% membership function
mem = zeros(gridsize,4);
for i = 1:gridsize
    if i==1
        mem(i,:) = [-inf -inf sample_v(i) sample_v(i+1)];
    elseif i == gridsize
        mem(i,:) = [sample_v(i-1) sample_v(i) inf inf];
    else
        mem(i,:) = [sample_v(i-1) sample_v(i) sample_v(i) sample_v(i+1)];
    end
end

save 'leg_trajectory_interpolation_sample.mat'


