clc;clear;close all 
%% test the correctness of Jacobian matrix 
load("G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot\com0_trajectory_reference_model_team1_follower1.mat");
load("G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\test_Leg\leg_trajectory_interpolation_sample.mat");
addpath 'G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot'

vl = r(11,:);
% phil = r(7,:);
% wl = r(8,:);
% afal = r(9,:);
%% Result: should use round(.) to elimate the computation error. 
%% 
q = zeros(8,length(vl));
dq = zeros(8,length(vl));
ddq = zeros(8,length(vl));
ACC = zeros(8,length(vl));
V = zeros(8,length(vl));
P = zeros(6,length(vl));
Tm_r_0 = zeros(4,4,length(vl));
Tm_l_0 = zeros(4,4,length(vl));
H = zeros(1,gridsize);
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
    Tm_r_0(:,:,i) = Tm_0*Tm_r_p;
    qr = leg_IK(Tm_r_0(:,:,i),L3,L4,L5);
    q2 = qr(1); q4 = qr(2); q6 = qr(3); q8 = qr(4);
    Tm_l_p = [[eye(3); 0 0 0] [xq_l;0;zq_l;1]];
    Tm_l_0(:,:,i) = Tm_0*Tm_l_p;
    ql = leg_IK(Tm_l_0(:,:,i),L3,L4,L5);
    q1 = ql(1); q3 = ql(2); q5 = ql(3); q7 = ql(4);
    % dq and ddq can be calculated by inverse Jacobian and differential of 
    % inverse Jacobian
    q(:,i) = [q1;q2;q3;q4;q5;q6;q7;q8];
    
    dq(:,i) = leg_Jb_inv(q(:,i),L4,L5)*[vx_r;0;vz_r;0;vx_l;0;vz_l;0]; % unit: rad/s
    
    ddq(:,i) = leg_Jb_inv(q(:,i),L4,L5)*[ax_r;0;az_r;0;ax_l;0;az_l;0]...
        + leg_dinvJbdt(dq(:,i),q(:,i),L4,L5)*[vx_r;0;vz_r;0;vx_l;0;vz_l;0];
    
    P(:,i) = [xq_r;0;zq_r;xq_l;0;zq_l];
    V(:,i) =  [vx_r;0;vz_r;0;vx_l;0;vz_l;0];
    ACC(:,i) =  [ax_r;0;az_r;0;ax_l;0;az_l;0];
    
    disp(i);
    toc;
end
save 'test.mat'
%%
load 'test.mat'
for i=1:8
figure;
plot(t(1:20000),q(i,1:20000));
end
for i=1:8
figure;
plot(t(1:20000),dq(i,1:20000));
end
for i=1:8
figure;
plot(t(1:20000),ddq(i,1:20000));
end
for i = 1:8
ddq_test(i,:) = [0,diff(dq(i,:))/dt];
end
for i = 1:8
figure;
plot(t(1:20000),ddq(i,1:20000),t(1:20000),ddq_test(i,1:20000))
end
% for i=1:6
% figure;
% plot(t,P(i,:));
% end
% for i=1:8
% figure;
% plot(t,V(i,:));
% end
% for i=1:8
% figure;
% plot(t,ACC(i,:));
% end
% 
% 
% %%
% dq_test = zeros(8,length(vl));
% for i = 1:8
% dq_test(i,:) = [0,diff(q(i,:))/dt];
% end
% %%
% for i=1:8
% figure;
% plot(t,q(i,:));
% end
% for i = 1:8
% figure;
% plot(t,dq(i,:),t,dq_test(i,:))
% end
% %%
% V_test = zeros(8,length(vl));
% for i = 1:length(vl)
%     V_test(:,i) = leg_Jb(q(:,i),L4,L5)*dq(:,i);
% end
% %%
% for i = 1:8
% figure; 
% plot(t,V(i,:),t,V_test(i,:));
% end
% %%
% Tm_r_0_test = zeros(4,4,length(vl));
% Tm_l_0_test = zeros(4,4,length(vl));
% P_test = zeros(6,length(vl));
% qr = [q(2,:);q(4,:);q(6,:);q(8,:)];
% ql = [q(1,:);q(3,:);q(5,:);q(7,:)];
% for i = 1:length(vl)
%     tic;
%     Tm_r_0_test(:,:,i) = leg_FK(qr(:,i),L3,L4,L5);
%     Tm_l_0_test(:,:,i) = leg_FK(ql(:,i),L3,L4,L5);
%     Pr=Tm_0\Tm_r_0_test(1:4,4,i);
%     Pl=Tm_0\Tm_l_0_test(1:4,4,i);
%     P_test(:,i) = [Pr(1:3);Pl(1:3)];
%     disp(i)
%     toc;
% end
% %%
% for i = 1:6
% figure; 
% plot(t,P(i,:),t,P_test(i,:));
% end
% %%
% ACC_test = zeros(8,length(vl));
% for i = 1:length(vl)
%     tic;
%     ACC_test(:,i) = leg_Jb(q(:,i),L4,L5)*ddq(:,i) + leg_dJbdt(dq(:,i),q(:,i),L4,L5)*dq(:,i);
%     disp(i)
%     toc;
% end
% %%
% for i = 1:8
% figure; 
% plot(t,ACC(i,:),t,ACC_test(i,:));
% end

%% movie 
clc;clear;close all
load 'test.mat'
L7 = 0.02;
z = 0.21; % 站立時的高度 
% d = 0.2485-z;
ct = 1;

for i = 1:500:length(vl)
    %%% right leg
    tic;
    xbr = r(1,i);
    ybr = r(2,i);
    zbr = z+L2+L6;
    xb1r = xbr;
    yb1r = ybr;
    zb1r = zbr - L2;
    x0r = xbr + L1*cos(-pi/2+r(7,i));
    y0r = ybr + L1*sin(-pi/2+r(7,i));
    z0r = zbr - L2;
    x1r = x0r;
    y1r = y0r;
    z1r = z0r-L3;
    x2r=x1r+L4*cos(-pi/2-q(6,i))*cos(r(7,i));
    y2r=y1r+L4*cos(-pi/2-q(6,i))*sin(r(7,i));
    z2r=z1r+L4*sin(-pi/2-q(6,i));
    x3r=x2r+L5*cos(-pi/2-q(6,i)-q(8,i))*cos(r(7,i));
    y3r=y2r+L5*cos(-pi/2-q(6,i)-q(8,i))*sin(r(7,i));
    z3r=z2r+L5*sin(-pi/2-q(6,i)-q(8,i));
    x4r=x3r+L6*cos(-pi/2)*cos(r(7,i));
    y4r=y3r+L6*cos(-pi/2)*sin(r(7,i));
    z4r=z3r+L6*sin(-pi/2);
    x5r=x4r+L7*cos(0)*cos(r(7,i));
    y5r=y4r+L7*cos(0)*sin(r(7,i));
    z5r=z4r+L7*sin(0);
    %%% left leg
    xbl = r(1,i);
    ybl = r(2,i);
    zbl = z+L2+L6;
    xb1l = xbl;
    yb1l = ybl;
    zb1l = zbl - L2;
    x0l = xbl + L1*cos(pi/2+r(7,i));
    y0l = ybl + L1*sin(pi/2+r(7,i));
    z0l = zbl - L2;
    x1l = x0l;
    y1l = y0l;
    z1l = z0l-L3;
    x2l=x1l+L4*cos(-pi/2-q(5,i))*cos(r(7,i));
    y2l=y1l+L4*cos(-pi/2-q(5,i))*sin(r(7,i));
    z2l=z1l+L4*sin(-pi/2-q(5,i));
    x3l=x2l+L5*cos(-pi/2-q(5,i)-q(7,i))*cos(r(7,i));
    y3l=y2l+L5*cos(-pi/2-q(5,i)-q(7,i))*sin(r(7,i));
    z3l=z2l+L5*sin(-pi/2-q(5,i)-q(7,i));
    x4l=x3l+L6*cos(-pi/2)*cos(r(7,i));
    y4l=y3l+L6*cos(-pi/2)*sin(r(7,i));
    z4l=z3l+L6*sin(-pi/2);
    x5l=x4l+L7*cos(0)*cos(r(7,i));
    y5l=y4l+L7*cos(0)*sin(r(7,i));
    z5l=z4l+L7*sin(0);
    
    plot3([xbr, xb1r, x0r, x1r, x2r, x3r, x4r, x5r],[ybr, yb1r, y0r, y1r, y2r, y3r, y4r, y5r],[zbr, zb1r, z0r, z1r, z2r, z3r, z4r, z5r],'LineWidth',0.5)
    hold on
    plot3([xbl, xb1l, x0l, x1l, x2l, x3l, x4l, x5l],[ybl, yb1l, y0l, y1l, y2l, y3l, y4l, y5l],[zbl, zb1l, z0l, z1l, z2l, z3l, z4l, z5l],'LineWidth',0.5)
    grid on 
    axis equal
    axis([3 18 1 9 0 0.5])
    hold off
%     axis equal
%     view([0 90])
%     axis([0 5 0 5 0 0.3312])
%     view([45 20])
%     pause(0.1)
    m(ct) = getframe(gcf);
    ct = ct+1;
    disp(i)
    toc;
end
% movie(m)
% videofile = VideoWriter('biped robot posture','uncompressed AVI');
% open(vediofile)
% writeVideo(videofile,m)
% close(vediofile)