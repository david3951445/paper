clc;clear;close all
load 'G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot\com0_trajectory_reference_model_team.mat'
load 'leg_trajectory_interpolation_sample.mat'
% addpath 'G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot'
%% defuzzy
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Problem: v 的點會多兩個 (因為使用diff)
% Solution: 在 xr 多設計兩個點 --> 繪圖時我們真正要的: length(v)-2

% Final Revision: 直接利用解析解去求 --> v在設計時不需要多兩個點
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vl1 = r(11,:);
vl2 = r(11+12,:);
vl3 = r(11+12*2,:);
vu1 = r(11+12*3,:);
vu2 = r(11+12*4,:);
vu3 = r(11+12*5,:);
phil1 = r(7,:);
wl1 = r(8,:);
afal1 = r(9,:);
phil2 = r(7+12,:);
wl2 = r(8+12,:);
afal2 = r(9+12,:);
phil3 = r(7+12*2,:);
wl3 = r(8+12*2,:);
afal3 = r(9+12*2,:);
phiu1 = r(7+12*3,:);
wu1 = r(8+12*3,:);
afau1 = r(9+12*3,:);
phiu2 = r(7+12*4,:);
wu2 = r(8+12*4,:);
afau2 = r(9+12*4,:);
phiu3 = r(7+12*5,:);
wu3 = r(8+12*5,:);
afau3 = r(9+12*5,:);

H = zeros(1,gridsize);
%% follower l1 
gait_f_l1 = zeros(36,length(vl1));
for i = 1:length(vl1)
    tic;
    if mod(i,len) ~= 0
        index = i - floor(i/len)*len;
    else 
        index = len;
    end
    for j = 1:gridsize
        H(j) = trapmf(vl1(i),mem(j,:));
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
gait_f_l1(13,:) = phil1; gait_f_l1(14,:) = phil1;
gait_f_l1(21,:) = wl1; gait_f_l1(22,:) = wl1;
gait_f_l1(29,:) = afal1; gait_f_l1(30,:) = afal1;

%% follower l2
gait_f_l2 = zeros(36,length(vl2));
for i = 1:length(vl2)
    tic;
    if mod(i,len) ~= 0
        index = i - floor(i/len)*len;
    else 
        index = len;
    end
    for j = 1:gridsize
        H(j) = trapmf(vl2(i),mem(j,:));
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
    gait_f_l2(:,i) =  [xq_r;vx_r;ax_r;zq_r;vz_r;az_r;xq_l;vx_l;ax_l;zq_l;vz_l;az_l;...
        q;dq;ddq];
    disp(i);
    toc;
end
%  In fact, q1~q4 沒有作用. 因此在這對q1和q2硬套上身體的轉向 
gait_f_l2(13,:) = phil2; gait_f_l2(14,:) = phil2;
gait_f_l2(21,:) = wl2; gait_f_l2(22,:) = wl2;
gait_f_l2(29,:) = afal2; gait_f_l2(30,:) = afal2;

%% follower l3
gait_f_l3 = zeros(36,length(vl3));
for i = 1:length(vl3)
    tic;
    if mod(i,len) ~= 0
        index = i - floor(i/len)*len;
    else 
        index = len;
    end
    for j = 1:gridsize
        H(j) = trapmf(vl3(i),mem(j,:));
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
    gait_f_l3(:,i) =  [xq_r;vx_r;ax_r;zq_r;vz_r;az_r;xq_l;vx_l;ax_l;zq_l;vz_l;az_l;...
        q;dq;ddq];
    disp(i);
    toc;
end
%  In fact, q1~q4 沒有作用. 因此在這對q1和q2硬套上身體的轉向 
gait_f_l3(13,:) = phil3; gait_f_l3(14,:) = phil3;
gait_f_l3(21,:) = wl3; gait_f_l3(22,:) = wl3;
gait_f_l3(29,:) = afal3; gait_f_l3(30,:) = afal3;

%% follower u1
gait_f_u1 = zeros(36,length(vu1));
for i = 1:length(vu1)
    tic;
    if mod(i,len) ~= 0
        index = i - floor(i/len)*len;
    else 
        index = len;
    end
    
    for j = 1:gridsize
        H(j) = trapmf(vu1(i),mem(j,:));
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
gait_f_u1(13,:) = phiu1; gait_f_u1(14,:) = phiu1;
gait_f_u1(21,:) = wu1; gait_f_u1(22,:) = wu1;
gait_f_u1(29,:) = afau1; gait_f_u1(30,:) = afau1;
%% follower u2
gait_f_u2 = zeros(36,length(vu2));
for i = 1:length(vu2)
    tic;
    if mod(i,len) ~= 0
        index = i - floor(i/len)*len;
    else 
        index = len;
    end
    
    for j = 1:gridsize
        H(j) = trapmf(vu2(i),mem(j,:));
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
    gait_f_u2(:,i) =  [xq_r;vx_r;ax_r;zq_r;vz_r;az_r;xq_l;vx_l;ax_l;zq_l;vz_l;az_l;...
        q;dq;ddq];
    disp(i);
    toc;
    
end
gait_f_u2(13,:) = phiu2; gait_f_u2(14,:) = phiu2;
gait_f_u2(21,:) = wu2; gait_f_u2(22,:) = wu2;
gait_f_u2(29,:) = afau2; gait_f_u2(30,:) = afau2;

%% follower u3
gait_f_u3 = zeros(36,length(vu3));
for i = 1:length(vu3)
    tic;
    if mod(i,len) ~= 0
        index = i - floor(i/len)*len;
    else 
        index = len;
    end
    
    for j = 1:gridsize
        H(j) = trapmf(vu3(i),mem(j,:));
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
    gait_f_u3(:,i) =  [xq_r;vx_r;ax_r;zq_r;vz_r;az_r;xq_l;vx_l;ax_l;zq_l;vz_l;az_l;...
        q;dq;ddq];
    disp(i);
    toc;
    
end
%%
gait_f_u3(13,:) = phiu3; gait_f_u3(14,:) = phiu3;
gait_f_u3(21,:) = wu3; gait_f_u3(22,:) = wu3;
gait_f_u3(29,:) = afau3; gait_f_u3(30,:) = afau3;
%% 用在畫圖測試
% save('leg_trajectory_interpolation_f_l1','t','gait_f_l1'); 
% save('leg_trajectory_interpolation_f_u1','t','gait_f_u1');

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

q9r_f_l2 = -gait_f_l2(15,:);
q10r_f_l2 = -gait_f_l2(16,:);
q11r_f_l2 = -gait_f_l2(17,:)-gait_f_l2(19,:);
q12r_f_l2 = -gait_f_l2(18,:)-gait_f_l2(20,:);
dq9r_f_l2 = -gait_f_l2(23,:);
dq10r_f_l2 = -gait_f_l2(24,:);
dq11r_f_l2 = -gait_f_l2(25,:)-gait_f_l2(27,:);
dq12r_f_l2 = -gait_f_l2(26,:)-gait_f_l2(28,:);
ddq9r_f_l2 = -gait_f_l2(31,:);
ddq10r_f_l2 = -gait_f_l2(32,:);
ddq11r_f_l2 = -gait_f_l2(33,:)-gait_f_l2(35,:);
ddq12r_f_l2 = -gait_f_l2(34,:)-gait_f_l2(36,:);

q9r_f_l3 = -gait_f_l3(15,:);
q10r_f_l3 = -gait_f_l3(16,:);
q11r_f_l3 = -gait_f_l3(17,:)-gait_f_l3(19,:);
q12r_f_l3 = -gait_f_l3(18,:)-gait_f_l3(20,:);
dq9r_f_l3 = -gait_f_l3(23,:);
dq10r_f_l3 = -gait_f_l3(24,:);
dq11r_f_l3 = -gait_f_l3(25,:)-gait_f_l3(27,:);
dq12r_f_l3 = -gait_f_l3(26,:)-gait_f_l3(28,:);
ddq9r_f_l3 = -gait_f_l3(31,:);
ddq10r_f_l3 = -gait_f_l3(32,:);
ddq11r_f_l3 = -gait_f_l3(33,:)-gait_f_l3(35,:);
ddq12r_f_l3 = -gait_f_l3(34,:)-gait_f_l3(36,:);

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

q9r_f_u2 = -gait_f_u2(15,:);
q10r_f_u2 = -gait_f_u2(16,:);
q11r_f_u2 = -gait_f_u2(17,:)-gait_f_u2(19,:);
q12r_f_u2 = -gait_f_u2(18,:)-gait_f_u2(20,:);
dq9r_f_u2 = -gait_f_u2(23,:);
dq10r_f_u2 = -gait_f_u2(24,:);
dq11r_f_u2 = -gait_f_u2(25,:)-gait_f_u2(27,:);
dq12r_f_u2 = -gait_f_u2(26,:)-gait_f_u2(28,:);
ddq9r_f_u2 = -gait_f_u2(31,:);
ddq10r_f_u2 = -gait_f_u2(32,:);
ddq11r_f_u2 = -gait_f_u2(33,:)-gait_f_u2(35,:);
ddq12r_f_u2 = -gait_f_u2(34,:)-gait_f_u2(36,:);

q9r_f_u3 = -gait_f_u3(15,:);
q10r_f_u3 = -gait_f_u3(16,:);
q11r_f_u3 = -gait_f_u3(17,:)-gait_f_u3(19,:);
q12r_f_u3 = -gait_f_u3(18,:)-gait_f_u3(20,:);
dq9r_f_u3 = -gait_f_u3(23,:);
dq10r_f_u3 = -gait_f_u3(24,:);
dq11r_f_u3 = -gait_f_u3(25,:)-gait_f_u3(27,:);
dq12r_f_u3 = -gait_f_u3(26,:)-gait_f_u3(28,:);
ddq9r_f_u3 = -gait_f_u3(31,:);
ddq10r_f_u3 = -gait_f_u3(32,:);
ddq11r_f_u3 = -gait_f_u3(33,:)-gait_f_u3(35,:);
ddq12r_f_u3 = -gait_f_u3(34,:)-gait_f_u3(36,:);

%%
save('leg_trajectory_qr_f_l1','q9r_f_l1','q10r_f_l1','q11r_f_l1','q12r_f_l1','dq9r_f_l1',...
    'dq10r_f_l1','dq11r_f_l1','dq12r_f_l1','ddq9r_f_l1','ddq10r_f_l1','ddq11r_f_l1','ddq12r_f_l1');
save('leg_trajectory_qr_f_l2','q9r_f_l2','q10r_f_l2','q11r_f_l2','q12r_f_l2','dq9r_f_l2',...
    'dq10r_f_l2','dq11r_f_l2','dq12r_f_l2','ddq9r_f_l2','ddq10r_f_l2','ddq11r_f_l2','ddq12r_f_l2');
save('leg_trajectory_qr_f_l3','q9r_f_l3','q10r_f_l3','q11r_f_l3','q12r_f_l3','dq9r_f_l3',...
    'dq10r_f_l3','dq11r_f_l3','dq12r_f_l3','ddq9r_f_l3','ddq10r_f_l3','ddq11r_f_l3','ddq12r_f_l3');

save('leg_trajectory_qr_f_u1','q9r_f_u1','q10r_f_u1','q11r_f_u1','q12r_f_u1','dq9r_f_u1',...
    'dq10r_f_u1','dq11r_f_u1','dq12r_f_u1','ddq9r_f_u1','ddq10r_f_u1','ddq11r_f_u1','ddq12r_f_u1');
save('leg_trajectory_qr_f_u2','q9r_f_u2','q10r_f_u2','q11r_f_u2','q12r_f_u2','dq9r_f_u2',...
    'dq10r_f_u2','dq11r_f_u2','dq12r_f_u2','ddq9r_f_u2','ddq10r_f_u2','ddq11r_f_u2','ddq12r_f_u2');
save('leg_trajectory_qr_f_u3','q9r_f_u3','q10r_f_u3','q11r_f_u3','q12r_f_u3','dq9r_f_u3',...
    'dq10r_f_u3','dq11r_f_u3','dq12r_f_u3','ddq9r_f_u3','ddq10r_f_u3','ddq11r_f_u3','ddq12r_f_u3');



