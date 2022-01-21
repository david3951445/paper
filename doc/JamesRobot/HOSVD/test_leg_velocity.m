%% need to change the code, since the unit of Jacobian matrix is changed.
load("G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot\com0_trajectory_reference_model_team1_follower1.mat");
load 'leg_trajectory_interpolation_sample.mat'

vl = r(11,:);
phil = r(7,:);
wl = r(8,:);
afal = r(9,:);

% vu = r(11+12,:);
% phiu = r(7+12,:);
% wu = r(8+12,:);
% afau = r(9+12,:);

H = zeros(1,gridsize);
%% follower l1 
gait_f_l = zeros(36,length(vl));
for i = 1:length(vl)
    tic;
    if mod(i,len) ~= 0
        index = i - floor(i/len)*len;
    else 
        index = len;
    end
    for j = 1:gridsize
        H(j) = trapmf(vl(i),mem(j,:));
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
    gait_f_l(:,i) =  [xq_r;vx_r;ax_r;zq_r;vz_r;az_r;xq_l;vx_l;ax_l;zq_l;vz_l;az_l;...
        q;dq;ddq];
    disp(i);
    toc;
end
%%
close all
figure;
plot(t,vl);
figure;
plot(t,gait_f_l(2,:));
figure;
plot(gait_f_l(1,:),gait_f_l(4,:))
figure;
plot(t,gait_f_l(21,:))
figure;
plot(t,gait_f_l(22,:))
figure;
plot(t,gait_f_l(23,:))
figure;
plot(t,gait_f_l(24,:))
figure;
plot(t,gait_f_l(25,:))
figure;
plot(t,gait_f_l(26,:))
figure;
plot(t,gait_f_l(27,:))
figure;
plot(t,gait_f_l(28,:))

figure;
plot(t,gait_f_l(29,:))
figure;
plot(t,gait_f_l(30,:))
figure;
plot(t,gait_f_l(31,:))
figure;
plot(t,gait_f_l(32,:))
figure;
plot(t,gait_f_l(33,:))
figure;
plot(t,gait_f_l(34,:))
figure;
plot(t,gait_f_l(35,:))
figure;
plot(t,gait_f_l(36,:))


% %  In fact, q1~q4 沒有作用. 因此在這對q1和q2硬套上身體的轉向 
% gait_f_l(13,:) = phil; gait_f_l(14,:) = phil;
% gait_f_l(21,:) = wl; gait_f_l(22,:) = wl;
% gait_f_l(29,:) = afal; gait_f_l(30,:) = afal;


%% follower u1
% gait_f_u = zeros(36,length(vu));
% for i = 1:length(vu)
%     tic;
%     if mod(i,len) ~= 0
%         index = i - floor(i/len)*len;
%     else 
%         index = len;
%     end
%     
%     for j = 1:gridsize
%         H(j) = trapmf(vu(i),mem(j,:));
%     end
%     
%     % 空間腳速度用內插
%     xq_r = H*sample_xq_r_p(:,index);
%     zq_r = H*sample_zq_r_p(:,index);
%     vx_r = H*sample_vx_r_b(:,index);
%     vz_r = H*sample_vz_r_b(:,index);
%     ax_r = H*sample_ax_r_b(:,index);
%     az_r = H*sample_az_r_b(:,index);
%     
%     xq_l = H*sample_xq_l_p(:,index);
%     zq_l = H*sample_zq_l_p(:,index);
%     vx_l = H*sample_vx_l_b(:,index);
%     vz_l = H*sample_vz_l_b(:,index);
%     ax_l = H*sample_ax_l_b(:,index);
%     az_l = H*sample_az_l_b(:,index);
%     % joint space 的直接算
%     Tm_r_p = [[eye(3); 0 0 0] [xq_r;0;zq_r;1]];
%     Tm_r_0 = Tm_0*Tm_r_p;
%     qr = leg_IK(Tm_r_0,L3,L4,L5);
%     q2 = qr(1); q4 = qr(2); q6 = qr(3); q8 = qr(4);
%     Tm_l_p = [[eye(3); 0 0 0] [xq_l;0;zq_l;1]];
%     Tm_l_0 = Tm_0*Tm_l_p;
%     ql = leg_IK(Tm_l_0,L3,L4,L5);
%     q1 = ql(1); q3 = ql(2); q5 = ql(3); q7 = ql(4);
%     % dq and ddq can be calculated by inverse Jacobian and differential of 
%     % inverse Jacobian
%     
%     % q1 and q2 is not from IK, it is from trajectory design (still need to think)
%     q = [q1;q2;q3;q4;q5;q6;q7;q8];
%     
%     dq = leg_Jb_inv(q,L4,L5)*[vx_r;0;vz_r;0;vx_l;0;vz_l;0];
%     
%     ddq = leg_Jb_inv(q,L4,L5)*[ax_r;0;az_r;0;ax_l;0;az_l;0]...
%         + leg_dinvJbdt(dq,q,L4,L5)*[vx_r;0;vz_r;0;vx_l;0;vz_l;0];
%     % 12 8 8 8 
%     gait_f_u(:,i) =  [xq_r;vx_r;ax_r;zq_r;vz_r;az_r;xq_l;vx_l;ax_l;zq_l;vz_l;az_l;...
%         q;dq;ddq];
%     disp(i);
%     toc;
%     
% end
% gait_f_u(13,:) = phiu; gait_f_u(14,:) = phiu;
% gait_f_u(21,:) = wu; gait_f_u(22,:) = wu;
% gait_f_u(29,:) = afau; gait_f_u(30,:) = afau;
% 
% 
% q9r_f_l = -gait_f_l(15,:);
% q10r_f_l = -gait_f_l(16,:);
% q11r_f_l = -gait_f_l(17,:)-gait_f_l(19,:);
% q12r_f_l = -gait_f_l(18,:)-gait_f_l(20,:);
% dq9r_f_l = -gait_f_l(23,:);
% dq10r_f_l = -gait_f_l(24,:);
% dq11r_f_l = -gait_f_l(25,:)-gait_f_l(27,:);
% dq12r_f_l = -gait_f_l(26,:)-gait_f_l(28,:);
% ddq9r_f_l = -gait_f_l(31,:);
% ddq10r_f_l = -gait_f_l(32,:);
% ddq11r_f_l = -gait_f_l(33,:)-gait_f_l(35,:);
% ddq12r_f_l = -gait_f_l(34,:)-gait_f_l(36,:);
% 
% q9r_f_u = -gait_f_u(15,:);
% q10r_f_u = -gait_f_u(16,:);
% q11r_f_u = -gait_f_u(17,:)-gait_f_u(19,:);
% q12r_f_u = -gait_f_u(18,:)-gait_f_u(20,:);
% dq9r_f_u = -gait_f_u(23,:);
% dq10r_f_u = -gait_f_u(24,:);
% dq11r_f_u = -gait_f_u(25,:)-gait_f_u(27,:);
% dq12r_f_u = -gait_f_u(26,:)-gait_f_u(28,:);
% ddq9r_f_u = -gait_f_u(31,:);
% ddq10r_f_u = -gait_f_u(32,:);
% ddq11r_f_u = -gait_f_u(33,:)-gait_f_u(35,:);
% ddq12r_f_u = -gait_f_u(34,:)-gait_f_u(36,:);
