clc;clear;close all

SS = load ("G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\RungeKutta\follower_tracking_data_team1_follower1_l.mat","tr","xbar");

load("G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot\com0_trajectory_reference_model_team1_follower1.mat");
load("G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\test_Leg\leg_trajectory_qr_team1_follower1_l.mat");
load ('G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\test_Leg\gain_F.mat');
LP3 = load('G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\test_Leg\lpv_data_part3_v4.mat');
S = load('G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\test_Leg\leg_trajectory_interpolation_sample.mat');

xS_domain = [LP3.domain];
xS_gridsize = [LP3.gridsize];
xS = cell(size(xS_domain,1),1);
U{1} = LP3.U{1};
U{2} = LP3.U{2};

% sample x
for i = 1:size(xS_domain,1)
    xS{i} = linspace(xS_domain(i,1),xS_domain(i,2),xS_gridsize(i));
end

LP3.P = size(LP3.domain,1);
numRule = prod(size(LP3.S,1:LP3.P));

for i = 1:numRule
    K(:,1:5,i) = zeros(4,5);
end
% K = 2*K;
c = 1;
for i = 1:length(tr)
    if i~= 1
        eV(c:c+1) = SS.xbar(19,i);
        c = c+2;
    else 
        eV(c) = SS.xbar(19,i);
        c = c+1;
    end
end

rbar = [eV+r(11,:);ddq9r_f_l;ddq10r_f_l;ddq11r_f_l;ddq12r_f_l];


xbar(1:5,1) = rbar(:,1);

xbar(6:9,1) = [0;0;0;0];

xbar(10:13,1) = [0;0;0;0];


% In this case, since the design method of IK, what we care are q5 q6 q7
% q8 (also on angular velocity)
% q11r = -(q5+q7), q12r = -(q6+q8)
% 7/27 since we have to show the figure on the paper, q1,q2,q3,q4 need to
% be considered here. 
q = zeros(8,length(tr));

Kbar1 = zeros(4,13,length(tr));
Kbar2 = zeros(4,13,length(tr));
Kbar3 = zeros(4,13,length(tr));

Len = S.len;
for i = 1:length(tr)-1
    tic;
    V = xbar(1,i);
    Index = i;

    Z1 = V;
    % Index is time-varying variable. Therefore, Kbar is also time-varying
    % gain
    Z2(1) = 2*Index-1; % tn
    Z2(2) = 2*Index; % tn+h/2
    Z2(3) = 2*Index+1; % tn+h
    
    for j = 1:3
        if mod(Z2(j),Len) ~= 0
            Z2(j) = Z2(j) - floor(Z2(j)/Len)*Len;
        else
            Z2(j) = Len;
        end
    end
    
    xmf1 = [Z1;Z2(1)];
    xmf2 = [Z1;Z2(2)];
    xmf3 = [Z1;Z2(3)];
    H1 = mf(xmf1,xS,U);
    H2 = mf(xmf2,xS,U);
    H3 = mf(xmf3,xS,U);
    Kbar1(:,:,i) = defuzzy(H1,K);
    Kbar2(:,:,i) = defuzzy(H2,K);
    Kbar3(:,:,i) = defuzzy(H3,K);
    
    % get the value of q
    q(:,i) = -f(V,Z2(1),S,0);
    
    k1 = RK(xbar(:,i),rbar(:,2*i-1),go,Kbar1(:,:,i),Z2(1),S);
    k2 = RK(xbar(:,i)+h*k1/2,rbar(:,2*i),go,Kbar2(:,:,i),Z2(2),S);
    k3 = RK(xbar(:,i)+h*k2/2,rbar(:,2*i),go,Kbar2(:,:,i),Z2(2),S);
    k4 = RK(xbar(:,i)+h*k3,rbar(:,2*i+1),go,Kbar3(:,:,i),Z2(3),S);
    xbar(:,i+1) = xbar(:,i) + 1/6*h*(k1+2*k2+2*k3+k4);
    
    if i == length(tr)-1
        V = xbar(1,i+1);
        Index = i+1;
        Index_q = 2*Index-1;
        if mod(Index_q,Len) ~= 0
            Index_q = Index_q - floor(Index_q/Len)*Len;
        else
            Index_q = Len;
        end
        q(:,i+1) = -f(V,Index_q,S,0);
    end
    
    disp(i);
    toc;
end
%% 
% for i = 1:13
%     figure;
%     plot(tr(1:10000),xbar(i,1:10000));
% end
for i = 1:8
figure;
plot(tr(1:20000),q(i,1:20000),tr(1:20000),q(i,1:20000)+xbar(i+5,1:20000));
end
%%
for i = 1:8
figure;
plot(tr(1:20000),xbar(i+5,1:20000));
end

function dx = RK(x,r,B,K,Index,S)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 實際：
beta = 10;
Ar = -beta*eye(5);
Br = beta*eye(5);


V = x(1);

dx(1:5,1) = Ar*x(1:5) + Br*r;
dx(6:9,1) = x(10:13);
dx(10:13,1) = f(V,Index,S,1) + x(2:5) + 0.1*randn(4,1); %% use another f(.)

dx = dx + B*K*x;
end

function Kbar = defuzzy(H,K)
c = 1;
Kbar = 0; 
for i1 = 1:size(H{1},2)
    for i2 = 1:size(H{2},2)
        h(c) = H{1}(i1)*H{2}(i2);
        c = c+1;
    end
end

for i = 1:c-1
    Kbar = h(i)*K(:,:,i) + Kbar;
end

end

function H = mf(x,xS,U)
    H = cell(1,size(U,2));
    n = zeros(1,size(U,2)); 
    for i = 1:size(U,2)
        n(i) = size(U{i},1); % number of sample point of each premise variable
    end
    
    for i = 1:size(U,2) % number of premise variable
        for j = 1:n(i)
            if x(i) <= xS{i}(1)
                H{i} = U{i}(1,:);
                break;
            elseif x(i) <= xS{i}(j)
                h1 = (x(i)-xS{i}(j-1))/(xS{i}(j)-xS{i}(j-1));
                h2 = 1-h1;
                %%% 分點公式 %%%
                H{i} = h1*U{i}(j,:) + h2*U{i}(j-1,:);
                break;
            elseif x(i) > xS{i}(n(i))
                H{i} = U{i}(n(i),:);
                break;
            end
        end
    end
end

function gait = f(V,index,S,flag)
% defuzzy 
for j = 1:S.gridsize
    H(j) = trapmf(V,S.mem(j,:));
end

if flag == 1
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
    
    gait = [ddq(3);ddq(4);ddq(5)+ddq(7);ddq(6)+ddq(8)];
else
    % 空間腳速度用內插
    xq_r = H*S.sample_xq_r_p(:,index);
    zq_r = H*S.sample_zq_r_p(:,index);
    vx_r = H*S.sample_vx_r_b(:,index);
    vz_r = H*S.sample_vz_r_b(:,index);
    
    xq_l = H*S.sample_xq_l_p(:,index);
    zq_l = H*S.sample_zq_l_p(:,index);
    vx_l = H*S.sample_vx_l_b(:,index);
    vz_l = H*S.sample_vz_l_b(:,index);
    
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
    
    gait = [q(3);q(4);q(5)+q(7);q(6)+q(8);dq(3);dq(4);dq(5)+dq(7);dq(6)+dq(8)];
end

end