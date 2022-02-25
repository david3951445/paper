clc;clear;close all
%% moving frames algorithm 

%% get q dq ddq
load("G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot\com0_trajectory_reference_model_team1_follower1.mat");
load("G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\Leg\leg_trajectory_interpolation_sample.mat");

V = xr(11,:);
% phil = r(7,:);
% wl = r(8,:);
% afal = r(9,:);

% vu = r(11+12,:);
% phiu = r(7+12,:);
% wu = r(8+12,:);
% afau = r(9+12,:);

H = zeros(1,gridsize);
%% 
q = zeros(12,length(V));
dq = zeros(12,length(V));
ddq = zeros(12,length(V));

for i = 1:length(V)
    tic;
    if mod(i,len) ~= 0
        index = i - floor(i/len)*len;
    else 
        index = len;
    end
    for j = 1:gridsize
        H(j) = trapmf(V(i),mem(j,:));
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
    q(1:8,i) = [q1;q2;q3;q4;q5;q6;q7;q8];
    
    dq(1:8,i) = leg_Jb_inv(q(1:8,i),L4,L5)*[vx_r;0;vz_r;0;vx_l;0;vz_l;0];
    
    ddq(1:8,i) = leg_Jb_inv(q(1:8,i),L4,L5)*[ax_r;0;az_r;0;ax_l;0;az_l;0]...
        + leg_dinvJbdt(dq(1:8,i),q(1:8,i),L4,L5)*[vx_r;0;vz_r;0;vx_l;0;vz_l;0];
    2*
    disp(i);
    toc;
end
% In fact, q1~q4 沒有作用. 因此在這對q1和q2硬套上身體的轉向 
%%% Problem: when we utilzing RNE, which one should we choose? %%%%%%%%%%%%
% 在計算 reference frame 的時候會有點奇怪  
% Right now, the paramter q1 q2 are obtained from IK. 

% q(1,:) = phil; q(2,:) = phil;
% dq(1,:) = wl; dq(2,:) = wl;
% ddq(1,:) = afal; ddq(2,:) = afal;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q(9,:) = -q(3,:);
q(10,:) = -q(4,:);
q(11,:) = -q(5,:)-q(7,:);
q(12,:) = -q(6,:)-q(8,:);

dq(9,:) = -dq(3,:);
dq(10,:) = -dq(4,:);
dq(11,:) = -dq(5,:)-dq(7,:);
dq(12,:) = -dq(6,:)-dq(8,:);

ddq(9,:) = -ddq(3,:);
ddq(10,:) = -ddq(4,:);
ddq(11,:) = -ddq(5,:)-ddq(7,:);
ddq(12,:) = -ddq(6,:)-ddq(8,:);
%% figure verification 
for i = 1:12
figure; 
plot(tr(1:30000),q(i,1:30000));
end
for i = 1:12
figure; 
plot(tr(1:30000),dq(i,1:30000));
end
for i = 1:12
figure; 
plot(tr(1:30000),ddq(i,1:30000));
end
%% Recursive Newton Euler Algorithm 
%% parameter setting 
% 先測試右腳
m2 = 0.243;
m4 = 1.045;
m6 = 3.095;
m8 = 2.401;
m10 = 1.045;
m12 = 0.223;

I2 = inertia(2,10,9,0,0,0)*1e-4;
I4 = inertia(6,17,17,0,0,0)*1e-4;
I6 = inertia(433,404,56,-3,29,20)*1e-4;
I8 = inertia(197,196,57,-3,29,14)*1e-4;
I10 = inertia(6,17,17,0,0,0)*1e-4;
I12 = inertia(22,99,91,0,-0.1,0)*1e-5;

% COMi is with respect to Frame{b} 
COM0 = [0;0;0];
COM2 = [-0.017;-0.035;-0.1157];
COM4 = [-0.073;-0.035;-0.1192];
COM6 = [0.017;-0.028;-0.3392];
COM8 = [-0.007;-0.031;-0.3972];
COM10 = [-0.016;-0.037;-0.4177];
COM12 = [-0.075;-0.035;-0.5222];

% Ai is the transformation matrix from based on Frame{i-1} to Frame{i}.
% 針對frame,先移動,再轉動
% 針對vector,先轉動,再移動
A1 = leg_T(q(2,1),-L3,0,pi/2);
A2 = leg_T(q(4,1)-pi/2,0,0,-pi/2);
A3 = leg_T(q(6,1),0,L4,0);
A4 = leg_T(q(8,1),0,L5,pi/2);
A5 = leg_T(q(10,1),0,0,-pi/2);
A6 = leg_T(q(12,1),0,L6,0);

R01 = A1(1:3,1:3);
R10 = R01';
R12 = A2(1:3,1:3);
R21 = R12';
R23 = A3(1:3,1:3);
R32 = R23';
R34 = A4(1:3,1:3);
R43 = R34';
R45 = A5(1:3,1:3);
R54 = R45';
R56 = A6(1:3,1:3);
R65 = R56';

eulb0 = [pi/2 0 0]; 
Rb0 = eul2rotm(eulb0);
R0b = Rb0';
Transb0 = [0;-L1;-L2];
Tb0 = [[Rb0 Transb0]; 0 0 0 1];

RF0 = Tb0(1:3,4); % with respect to Frame{b}

g = -9.81;
w0 = [0;0;0];
z0 = [0;0;1];
dw0 = [0;0;0];
a0 = [0;0;g];
r01 = A1(1:3,4); % with respect to Frame{0}
r0c1 = R0b*(COM2-RF0); % with respect to Frame{0}
r1c1 = r0c1-r01; % with respect to Frame{0}

z1 = [0;0;1];
r12 = A2(1:3,4); % with respect to Frame{1}
RF1 = RF0+Rb0*r01; % with respect to Frame{b}
r1c2 = R10*R0b*(COM4-RF1); % with respect to Frame{1}
r2c2 = r1c2-r12; % with respect to Frame{1}

z2 = [0;0;1];
r23 = A3(1:3,4); % with respect to Frame{2}
RF2 = RF1+Rb0*R01*r12; % with respect to Frame{b}
r2c3 = R21*R10*R0b*(COM6-RF2); % with respect to Frame{2}
r3c3 = r2c3-r23; % with respect to Frame{2}

z3 = [0;0;1];
r34 = A4(1:3,4); % with respect to Frame{3}
RF3 = RF2+Rb0*R01*R12*r23; % with respect to Frame{b}
r3c4 = R32*R21*R10*R0b*(COM8-RF3); % with respect to Frame{3}
r4c4 = r3c4-r34; % with respect to Frame{3}

z4 = [0;0;1];
r45 = A5(1:3,4); % with respect to Frame{4}
RF4 = RF3+Rb0*R01*R12*R23*r34; % with respect to Frame{b}
r4c5 = R43*R32*R21*R10*R0b*(COM10-RF4); % with respect to Frame{4}
r5c5 = r4c5-r45; % with respect to Frame{4}

z5 = [0;0;1];
r56 = A6(1:3,4); % with respect to Frame{5}
RF5 = RF4+Rb0*R01*R12*R23*R34*r45; % with respect to Frame{b}
r5c6 = R54*R43*R32*R21*R10*R0b*(COM12-RF5); % with respect to Frame{5}
r6c6 = r5c6-r56; % with respect to Frame{5}

tau = zeros(6,length(V));


w1 = zeros(3,length(V));
w2 = zeros(3,length(V));
w3 = zeros(3,length(V));
w4 = zeros(3,length(V));
w5 = zeros(3,length(V));
w6 = zeros(3,length(V));
dw1 = zeros(3,length(V));
dw2 = zeros(3,length(V));
dw3 = zeros(3,length(V));
dw4 = zeros(3,length(V));
dw5 = zeros(3,length(V));
dw6 = zeros(3,length(V));
a1 = zeros(3,length(V));
a2 = zeros(3,length(V));
a3 = zeros(3,length(V));
a4 = zeros(3,length(V));
a5 = zeros(3,length(V));
a6 = zeros(3,length(V));
ac1 = zeros(3,length(V));
ac2 = zeros(3,length(V));
ac3 = zeros(3,length(V));
ac4 = zeros(3,length(V));
ac5 = zeros(3,length(V));
ac6 = zeros(3,length(V));
f1 = zeros(3,length(V));
f2 = zeros(3,length(V));
f3 = zeros(3,length(V));
f4 = zeros(3,length(V));
f5 = zeros(3,length(V));
f6 = zeros(3,length(V));
n1 = zeros(3,length(V));
n2 = zeros(3,length(V));
n3 = zeros(3,length(V));
n4 = zeros(3,length(V));
n5 = zeros(3,length(V));
n6 = zeros(3,length(V));
%% 
for i = 1:length(V)
    tic;
    A1 = leg_T(q(2,i),-L3,0,pi/2);
    A2 = leg_T(q(4,i)-pi/2,0,0,-pi/2);
    A3 = leg_T(q(6,i),0,L4,0);
    A4 = leg_T(q(8,i),0,L5,pi/2);
    A5 = leg_T(q(10,i),0,0,-pi/2);
    A6 = leg_T(q(12,i),0,L6,0);
    
    R01 = A1(1:3,1:3);
    R10 = R01';
    R12 = A2(1:3,1:3);
    R21 = R12';
    R23 = A3(1:3,1:3);
    R32 = R23';
    R34 = A4(1:3,1:3);
    R43 = R34';
    R45 = A5(1:3,1:3);
    R54 = R45';
    R56 = A6(1:3,1:3);
    R65 = R56';
    
    dq1 = dq(2,i);
    dq2 = dq(4,i);
    dq3 = dq(6,i);
    dq4 = dq(8,i);
    dq5 = dq(10,i);
    dq6 = dq(12,i);
    ddq1 = ddq(2,i);
    ddq2 = ddq(4,i);
    ddq3 = ddq(6,i);
    ddq4 = ddq(8,i);
    ddq5 = ddq(10,i);
    ddq6 = ddq(12,i);
    
    w1(:,i) = R10*(w0+dq1*z0);
    dw1(:,i) = R10*(dw0+ddq1*z0+dq1*cross(w0,z0));
    a1(:,i) = R10*a0 + cross(dw1(:,i),R10*r01) + cross(w1(:,i),cross(w1(:,i),R10*r01));
    ac1(:,i) = a1(:,i) + cross(dw1(:,i),r1c1) + cross(w1(:,i),cross(w1(:,i),r1c1));
    
    w2(:,i) = R21*(w1(:,i)+dq2*z1);
    dw2(:,i) = R21*(dw1(:,i)+ddq2*z1+dq2*cross(w1(:,i),z1));
    a2(:,i) = R21*a1(:,i) + cross(dw2(:,i),R21*r12) + cross(w2(:,i),cross(w2(:,i),R21*r12));
    ac2(:,i) = a2(:,i) + cross(dw2(:,i),r2c2) + cross(w2(:,i),cross(w2(:,i),r2c2));

    w3(:,i) = R32*(w2(:,i)+dq3*z2);
    dw3(:,i) = R32*(dw2(:,i)+ddq3*z2+dq3*cross(w2(:,i),z2));
    a3(:,i) = R32*a2(:,i) + cross(dw3(:,i),R32*r23) + cross(w3(:,i),cross(w3(:,i),R32*r23));
    ac3(:,i) = a3(:,i) + cross(dw3(:,i),r3c3) + cross(w3(:,i),cross(w3(:,i),r3c3));
    
    w4(:,i) = R43*(w3(:,i)+dq4*z3);
    dw4(:,i) = R43*(dw3(:,i)+ddq4*z3+dq4*cross(w3(:,i),z3));
    a4(:,i) = R43*a3(:,i) + cross(dw4(:,i),R43*r34) + cross(w4(:,i),cross(w4(:,i),R43*r34));
    ac4(:,i) = a4(:,i) + cross(dw4(:,i),r4c4) + cross(w4(:,i),cross(w4(:,i),r4c4));
    
    w5(:,i) = R54*(w4(:,i)+dq5*z4);
    dw5(:,i) = R54*(dw4(:,i)+ddq5*z4+dq5*cross(w4(:,i),z4));
    a5(:,i) = R54*a4(:,i) + cross(dw5(:,i),R54*r45) + cross(w5(:,i),cross(w5(:,i),R54*r45));
    ac5(:,i) = a5(:,i) + cross(dw5(:,i),r5c5) + cross(w5(:,i),cross(w5(:,i),r5c5));
    
    w6(:,i) = R65*(w5(:,i)+dq6*z5);
    dw6(:,i) = R65*(dw5(:,i)+ddq6*z5+dq6*cross(w5(:,i),z5));
    a6(:,i) = R65*a5(:,i) + cross(dw6(:,i),R65*r56) + cross(w6(:,i),cross(w6(:,i),R65*r56));
    ac6(:,i) = a6(:,i) + cross(dw6(:,i),r6c6) + cross(w6(:,i),cross(w6(:,i),r6c6));
    
    f7 = [0;0;0];
    n7 = [0;0;0];
        
    f6(:,i) = f7 + m12*ac6(:,i);
    n6(:,i) = n7 - cross(f6(:,i),R65*(r56+r6c6)) + cross(f7,R65*r6c6) + I12*dw6(:,i) + cross(w6(:,i),(I12*w6(:,i)));
    tau(6,i) = n6(:,i)'*(R65*z5);
    
    f5(:,i) = R56*f6(:,i) + m10*ac5(:,i);
    n5(:,i) = n6(:,i) - cross(f5(:,i),R54*(r45+r5c5)) + cross(R56*f6(:,i),R54*r5c5) + I10*dw5(:,i) + cross(w5(:,i),(I10*w5(:,i)));
    tau(5,i) = n5(:,i)'*(R54*z4);
    
    f4(:,i) = R45*f5(:,i) + m8*ac4(:,i);
    n4(:,i) = n5(:,i) - cross(f4(:,i),R43*(r34+r4c4)) + cross(R45*f5(:,i),R43*r4c4) + I8*dw4(:,i) + cross(w4(:,i),(I8*w4(:,i)));
    tau(4,i) = n4(:,i)'*(R43*z3);
    
    f3(:,i) = R34*f4(:,i) + m6*ac3(:,i);
    n3(:,i) = n4(:,i) - cross(f3(:,i),R32*(r23+r3c3)) + cross(R34*f4(:,i),R32*r3c3) + I6*dw3(:,i) + cross(w3(:,i),(I6*w3(:,i)));
    tau(3,i) = n3(:,i)'*(R32*z2);
    
    f2(:,i) = R23*f3(:,i) + m4*ac2(:,i);
    n2(:,i) = n3(:,i) - cross(f2(:,i),R21*(r12+r2c2)) + cross(R23*f3(:,i),R21*r2c2) + I4*dw2(:,i) + cross(w2(:,i),(I4*w2(:,i)));
    tau(2,i) = n2(:,i)'*(R21*z1);
    
    f1(:,i) = R12*f2(:,i) + m2*ac1(:,i);
    n1(:,i) = n2(:,i) - cross(f1(:,i),R10*(r01+r1c1)) + cross(R12*f2(:,i),R10*r1c1) + I2*dw1(:,i) + cross(w1(:,i),(I2*w1(:,i)));
    tau(1,i) = n1(:,i)'*(R10*z0);
    
    disp(i);
    toc;
end
save('RNE_tau.mat','tau');
save('RNE_q.mat','q','dq','ddq');
%% tau figure
figure; 
plot(tr(1:30000),tau(1,1:30000));
figure; 
plot(tr(1:30000),tau(2,1:30000));
figure; 
plot(tr(1:30000),tau(3,1:30000));
figure; 
plot(tr(1:30000),tau(4,1:30000));
figure; 
plot(tr(1:30000),tau(5,1:30000));
figure; 
plot(tr(1:30000),tau(6,1:30000));
%% Remark: the control input will not be zero
figure; 
plot(tr,tau(1,:));
figure; 
plot(tr,tau(2,:));
figure; 
plot(tr,tau(3,:));
figure; 
plot(tr,tau(4,:));
figure; 
plot(tr,tau(5,:));
figure; 
plot(tr,tau(6,:));
function I = inertia(Ixx,Iyy,Izz,Ixy,Ixz,Iyz)
    I = [
        Ixx, Ixy, Ixz;
        Ixy, Iyy, Iyz;
        Ixz, Iyz, Izz
        ];
end
