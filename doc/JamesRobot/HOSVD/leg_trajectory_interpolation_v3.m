clc;clear;close all
load 'G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot\com0_trajectory_reference_model_f1_v1.mat'
addpath 'G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot'
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 座標轉換
% inverse Kinematics: the position of leg is relative to frame{0} 
% inverse Jacobian: the velocity of leg is relative to frame{b}

% Remark: In this version, there are two followers.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 撒點
domain = [0,0.25];
gridsize = 25;
sample_v = linspace(domain(1),domain(2),gridsize);
sample_v = sample_v';

%% parameter
%% design the number of step of robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% when time period of one gait is 2*T 
% 0 ~ 2T is matching to pi ~ -pi
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
step = 20;
Tp = t_end/step; % time period of a single step
tp = 0:hh:2*Tp-hh; % time instant of a complete step (2 single step) 
len = length(tp); 
hf = 0.02; % the height of gait
c = 0.25; % the constant multiply to v --> deteremine the gait length
dt = hh;

% link length
L1 = 0.035;
L2 = 0.0907;
L3 = 0.0285;
L4 = 0.11;
L5 = 0.11;

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
%         if sample_zq_r_p(i,j+1) < 1e-12
%             sample_zq_r_p(i,j+1) = 0;
%         end
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Joint space 的直接算，不用內插
% 不用 Jacobian matrix ，用diff(.)
% Problem: ddq 利用 leg_dinvJbdt 算出來的和 diff(.) 的不同 --> Solved
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% %% generate right gait (Joint)
% addpath 'G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot'
% 
% % transformation matrix on frame{p} 
% sample_Tm_r_p = zeros(4,4,len,gridsize);
% for i = 1:gridsize
%     for j = 1:len
%         sample_Tm_r_p(:,:,j,i) = [[eye(3); 0 0 0] sample_trans_r_p(:,j,i)];
%     end
% end
% 
%%% Coordinate Transformation (座標轉換) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% % d_tool = [-90 0 90]; %zyx
% % r_tool = deg2rad(d_tool);
% % rotm_tool = eul2rotm(r_tool);
% % d_base = [90 0 0]; %zyx
% % r_base = deg2rad(d_base);
% % rotm_base = eul2rotm(r_base);
% % % output small value to 0 
% % for i = 1:3
% %     for j = 1:3
% %         if abs(rotm_tool(i,j)) < 1e-6
% %             rotm_tool(i,j) = 0;
% %         end
% %         if abs(rotm_base(i,j)) < 1e-6
% %             rotm_base(i,j) = 0;
% %         end
% %     end
% % end
% % A_base_r = cat(2,cat(1,rotm_base,zeros(1,3)),[0;-L1;-L2;1]); % frame{b} to frame{0}
% % A_tool_r = cat(2,cat(1,rotm_tool,zeros(1,3)),[0;0;0;1]); % frame{4} to frame{tool}
% % Tr = A_base_r*A*A_tool_r; % from frame{b} to frame{tool}
% % T0r = A_base_r; % frame{0} is respect to frame{b} --> motor 1
% % T1r = A_base_r*A1; % frame{1} is respect to frame{b} --> motor 2
% % T2r = A_base_r*A1*A2; % frame{2} is respect to frame{b} --> motor 3
% % T3r = A_base_r*A1*A2*A3; % frame{3} is respect to frame{b} --> motor 4
% % T4r = A_base_r*A;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 座標轉換: frame{0} to frame{p} 
% Reason: 軌跡規劃的座標是frame{p},要轉換到frame{0}才能做IK
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sample_Tm_r_0 = zeros(4,4,len,gridsize);
% for i = 1:gridsize
%     for j = 1:len
%         sample_Tm_r_0(:,:,j,i) = Tm_0*sample_Tm_r_p(:,:,j,i);
%     end
% end
% 
% sample_q_r = zeros(4,len,gridsize);
% for i = 1:gridsize
%     for j = 1:len
%         sample_q_r(:,j,i) = leg_IK(sample_Tm_r_0(:,:,j,i),L3,L4,L5);
%     end
% end
% %% generate left gait (Joint)
% sample_Tm_l_p = zeros(4,4,len,gridsize);
% for i = 1:gridsize
%     for j = 1:len
%         sample_Tm_l_p(:,:,j,i) = [[eye(3); 0 0 0] sample_trans_r_p(:,j,i)];
%     end
% end
% 
% sample_Tm_l_0 = zeros(4,4,len,gridsize);
% for i = 1:gridsize
%     for j = 1:len
%         sample_Tm_l_0(:,:,j,i) = Tm_0*sample_Tm_l_p(:,:,j,i);
%     end
% end
% 
% sample_q_l = zeros(4,len,gridsize);
% for i = 1:gridsize
%     for j = 1:len
%         sample_q_l(:,j,i) = leg_IK(sample_Tm_l_0(:,:,j,i),L3,L4,L5);
%     end
% end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Problem: inverse Jacobian 推出來的數值和 diff 推出來的不一樣
% Solution: 用 inverse jacobian 解出來的 dq 的單位是 (rad/s)
% 因此，diff(.) 出來的數值要再 *pi/180 就會跟 inverse jacobian 的一樣
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% vleg,r & vleg,l = [vx;vy;vz;w];  
% vleg = [vleg,r;vleg,l]; 
% q = [q1,q2,q3,q4,q5,q6,q7,q8]'
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sample_v_leg_r = zeros(4,len,gridsize);
% sample_v_leg_l = zeros(4,len,gridsize);
% sample_v_leg = zeros(8,len,gridsize);
% sample_q = zeros(8,len,gridsize);
% for i = 1:gridsize
%     sample_v_leg_r(:,:,i) = [sample_vx_r_b(i,:);zeros(1,len);sample_vz_r_b(i,:);zeros(1,len)]; 
%     sample_v_leg_l(:,:,i) = [sample_vx_l_b(i,:);zeros(1,len);sample_vz_l_b(i,:);zeros(1,len)];
%     sample_v_leg(:,:,i) = [sample_v_leg_r(:,:,i);sample_v_leg_l(:,:,i)];
%     sample_q(:,:,i) = [sample_q_l(1,:,i);sample_q_r(1,:,i);sample_q_l(2,:,i);sample_q_r(2,:,i);...
%         sample_q_l(3,:,i);sample_q_r(3,:,i);sample_q_l(4,:,i);sample_q_r(4,:,i)];
% end
% 
% sample_dq = zeros(8,len,gridsize);
% for i = 1:gridsize
%     for j = 1:len
%         sample_dq(:,j,i) = leg_Jb_inv(sample_q(:,j,i),L4,L5)*sample_v_leg(:,j,i); % unit: rad/s
%         fprintf('1. %g,%g\n',i,j);
%     end
% end
% %% generate gait (ddq = inv(J)*ddx + d{inv(J)}*dx)
% sample_a_leg_r = zeros(4,len,gridsize);
% sample_a_leg_l = zeros(4,len,gridsize);
% sample_a_leg = zeros(8,len,gridsize);
% for i = 1:gridsize
%     sample_a_leg_r(:,:,i) = [sample_ax_r_b(i,:);zeros(1,len);sample_az_r_b(i,:);zeros(1,len)]; 
%     sample_a_leg_l(:,:,i) = [sample_ax_l_b(i,:);zeros(1,len);sample_az_l_b(i,:);zeros(1,len)];
%     sample_a_leg(:,:,i) = [sample_a_leg_r(:,:,i);sample_a_leg_l(:,:,i)];
% end
% 
% % sample_dq_deg = sample_dq*180/pi; % rad --> degree
% sample_ddq = zeros(8,len,gridsize);
% for i = 1:gridsize
%     for j = 1:len
%         sample_ddq(:,j,i) = leg_Jb_inv(sample_q(:,j,i),L4,L5)*sample_a_leg(:,j,i) ...
%             + leg_dinvJbdt(sample_dq(:,j,i),sample_q(:,j,i),L4,L5)*sample_v_leg(:,j,i);
%         fprintf('2. %g,%g\n',i,j);
%     end
% end

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

%% defuzzy
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Problem: v 的點會多兩個 (因為使用diff)
% Solution: 在 xr 多設計兩個點 --> 繪圖時我們真正要的: length(v)-2

% Final Revision: 直接利用解析解去求 --> v在設計時不需要多兩個點
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
H = zeros(1,gridsize);
%% follower 1 
gait_f_l1 = zeros(36,length(vr));
for i = 1:length(vr)
    tic;
    if mod(i,len) ~= 0
        index = i - floor(i/len)*len;
    else 
        index = len;
    end
    for j = 1:gridsize
        H(j) = trapmf(vr(i),mem(j,:));
    end
    % 空間腳速度用內插
    xq_r = H*sample_xq_r_p(:,index);
    zq_r = H*sample_zq_r_p(:,index);
    vx_r = H*sample_vx_r_b(:,index);
    vz_r = H*sample_vz_r_b(:,index);
    ax_r = H*sample_ax_r_b(:,index);
    az_r = H*sample_az_r_b(:,index);
    
    xq_l = H*sample_xq_l_p(:,index);
    zq_l = H*sample_zq_l_p(:,index);
    vx_l = H*sample_vx_l_b(:,index);
    vz_l = H*sample_vz_l_b(:,index);
    ax_l = H*sample_ax_l_b(:,index);
    az_l = H*sample_az_l_b(:,index);
    % joint space 的直接算
    Tm_r_p = [[eye(3); 0 0 0] [xq_r;0;zq_r;1]];
    Tm_r_0 = Tm_0*Tm_r_p;
    qr = leg_IK(Tm_r_0,L3,L4,L5);
    q2 = qr(1); q4 = qr(2); q6 = qr(3); q8 = qr(4);
    Tm_l_p = [[eye(3); 0 0 0] [xq_l;0;zq_l;1]];
    Tm_l_0 = Tm_0*Tm_l_p;
    ql = leg_IK(Tm_l_0,L3,L4,L5);
    q1 = ql(1); q3 = ql(2); q5 = ql(3); q7 = ql(4);
    % dq and ddq can be calculated by inverse Jacobian and differential of 
    % inverse Jacobian
    
    % q1 and q2 is not from IK, it is from trajectory design (still need to think)
    q = [q1;q2;q3;q4;q5;q6;q7;q8];
    
    dq = leg_Jb_inv(q,L4,L5)*[vx_r;0;vz_r;0;vx_l;0;vz_l;0];
    
    ddq = leg_Jb_inv(q,L4,L5)*[ax_r;0;az_r;0;ax_l;0;az_l;0]...
        + leg_dinvJbdt(dq,q,L4,L5)*[vx_r;0;vz_r;0;vx_l;0;vz_l;0];
    % 12 8 8 8 
    gait_f_l1(:,i) =  [xq_r;vx_r;ax_r;zq_r;vz_r;az_r;xq_l;vx_l;ax_l;zq_l;vz_l;az_l;...
        q;dq;ddq];
    disp(i);
    toc;
end
%  In fact, q1~q4 沒有作用. 因此在這對q1和q2硬套上身體的轉向 
gait_f_l1(13,:) = phir; gait_f_l1(14,:) = phir;
gait_f_l1(21,:) = wr; gait_f_l1(22,:) = wr;
gait_f_l1(29,:) = afar; gait_f_l1(30,:) = afar;


%% follower 2
gait_f_u1 = zeros(36,length(pjvr));
for i = 1:length(pjvr)
    tic;
    if mod(i,len) ~= 0
        index = i - floor(i/len)*len;
    else 
        index = len;
    end
    
    for j = 1:gridsize
        H(j) = trapmf(pjvr(i),mem(j,:));
    end
    
    % 空間腳速度用內插
    xq_r = H*sample_xq_r_p(:,index);
    zq_r = H*sample_zq_r_p(:,index);
    vx_r = H*sample_vx_r_b(:,index);
    vz_r = H*sample_vz_r_b(:,index);
    ax_r = H*sample_ax_r_b(:,index);
    az_r = H*sample_az_r_b(:,index);
    
    xq_l = H*sample_xq_l_p(:,index);
    zq_l = H*sample_zq_l_p(:,index);
    vx_l = H*sample_vx_l_b(:,index);
    vz_l = H*sample_vz_l_b(:,index);
    ax_l = H*sample_ax_l_b(:,index);
    az_l = H*sample_az_l_b(:,index);
    % joint space 的直接算
    Tm_r_p = [[eye(3); 0 0 0] [xq_r;0;zq_r;1]];
    Tm_r_0 = Tm_0*Tm_r_p;
    qr = leg_IK(Tm_r_0,L3,L4,L5);
    q2 = qr(1); q4 = qr(2); q6 = qr(3); q8 = qr(4);
    Tm_l_p = [[eye(3); 0 0 0] [xq_l;0;zq_l;1]];
    Tm_l_0 = Tm_0*Tm_l_p;
    ql = leg_IK(Tm_l_0,L3,L4,L5);
    q1 = ql(1); q3 = ql(2); q5 = ql(3); q7 = ql(4);
    % dq and ddq can be calculated by inverse Jacobian and differential of 
    % inverse Jacobian
    
    % q1 and q2 is not from IK, it is from trajectory design (still need to think)
    q = [q1;q2;q3;q4;q5;q6;q7;q8];
    
    dq = leg_Jb_inv(q,L4,L5)*[vx_r;0;vz_r;0;vx_l;0;vz_l;0];
    
    ddq = leg_Jb_inv(q,L4,L5)*[ax_r;0;az_r;0;ax_l;0;az_l;0]...
        + leg_dinvJbdt(dq,q,L4,L5)*[vx_r;0;vz_r;0;vx_l;0;vz_l;0];
    % 12 8 8 8 
    gait_f_u1(:,i) =  [xq_r;vx_r;ax_r;zq_r;vz_r;az_r;xq_l;vx_l;ax_l;zq_l;vz_l;az_l;...
        q;dq;ddq];
    disp(i);
    toc;
    
end
%%
gait_f_u1(13,:) = pjphir; gait_f_u1(14,:) = pjphir;
gait_f_u1(21,:) = pjwr; gait_f_u1(22,:) = pjwr;
gait_f_u1(29,:) = pjafar; gait_f_u1(30,:) = pjafar;

%%
save('leg_trajectory_interpolation_f_l1','t','gait_f_l1'); % 用在畫圖 
save('leg_trajectory_interpolation_f_u1','t','gait_f_u1');

%%
q9r_f_l1 = -gait_f_l1(15,:);
q10r_f_l1 = -gait_f_l1(16,:);
q11r_f_l1 = -gait_f_l1(17,:)-gait_f_l1(19,:);
q12r_f_l1 = -gait_f_l1(18,:)-gait_f_l1(20,:);

dq9r_f_l1 = -gait_f_l1(23,:);
dq10r_f_l1 = -gait_f_l1(24,:);
dq11r_f_l1 = -gait_f_l1(25,:)-gait_f_l1(27,:);
dq12r_f_l1 = -gait_f_l1(26,:)-gait_f_l1(28,:);

ddq9r_f_l1 = -gait_f_l1(31,:);
ddq10r_f_l1 = -gait_f_l1(32,:);
ddq11r_f_l1 = -gait_f_l1(33,:)-gait_f_l1(35,:);
ddq12r_f_l1 = -gait_f_l1(34,:)-gait_f_l1(36,:);

q9r_f_u1 = -gait_f_u1(15,:);
q10r_f_u1 = -gait_f_u1(16,:);
q11r_f_u1 = -gait_f_u1(17,:)-gait_f_u1(19,:);
q12r_f_u1 = -gait_f_u1(18,:)-gait_f_u1(20,:);

dq9r_f_u1 = -gait_f_u1(23,:);
dq10r_f_u1 = -gait_f_u1(24,:);
dq11r_f_u1 = -gait_f_u1(25,:)-gait_f_u1(27,:);
dq12r_f_u1 = -gait_f_u1(26,:)-gait_f_u1(28,:);

ddq9r_f_u1 = -gait_f_u1(31,:);
ddq10r_f_u1 = -gait_f_u1(32,:);
ddq11r_f_u1 = -gait_f_u1(33,:)-gait_f_u1(35,:);
ddq12r_f_u1 = -gait_f_u1(34,:)-gait_f_u1(36,:);

%%
save('leg_trajectory_qr_f_l1','q9r_f_l1','q10r_f_l1','q11r_f_l1','q12r_f_l1','dq9r_f_l1',...
    'dq10r_f_l1','dq11r_f_l1','dq12r_f_l1','ddq9r_f_l1','ddq10r_f_l1','ddq11r_f_l1','ddq12r_f_l1');
save('leg_trajectory_qr_f_u1','q9r_f_u1','q10r_f_u1','q11r_f_u1','q12r_f_u1','dq9r_f_u1',...
    'dq10r_f_u1','dq11r_f_u1','dq12r_f_u1','ddq9r_f_u1','ddq10r_f_u1','ddq11r_f_u1','ddq12r_f_u1');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dq, ddq
% 測試用diff和inverse Jacobian的差別
% Conclusion: 有些許不同，但大致上是相同的
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% gait_test = zeros(8,length(v));
% for i = 1:8
%     gait_test(i,2:end) = diff(gait(i+12,:))/dt;
% end
% gait_test_2 = zeros(8,length(v));
% for i = 1:8
%     gait_test_2(i,2:end) = diff(gait_test(i,:))/dt;
% end
% %%
% gait_test = zeros(8,length(v));
% for i = 1:length(v)
%     gait_test(:,i) = leg_Jb_inv(gait(13:20,i),L4,L5)*[gait(2,i);0;gait(5,i);0;gait(8,i);0;gait(11,i);0];
% end
% %%
% gait_test_2 = zeros(8,length(v));
% for i = 1:length(v)
%     gait_test_2(:,i) = leg_Jb_inv(gait(13:20,i),L4,L5)*[gait(3,i);0;gait(6,i);0;gait(9,i);0;gait(12,i);0]...
%         + leg_dinvJbdt(gait_test(:,i),gait(13:20,i),L4,L5)*[gait(2,i);0;gait(5,i);0;gait(8,i);0;gait(11,i);0];
% end



