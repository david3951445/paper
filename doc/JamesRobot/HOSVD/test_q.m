%%
clc;clear;close all
addpath 'G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot'
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 測試固定速度下的q dq ddq的變化狀況
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% parameter
v = 0.12;
Tp = 2; % time period of a single step
hr = 1e-3; 
tp = 0:hr:2*Tp-hr; % time instant of a complete step (2 single step)
len = length(tp);
hf = 0.02; % the height of gait
c = 1; % the constant multiply to v --> deteremine the gait length
dt = hr;


% link length
L1 = 0.035;
L2 = 0.0907;
L3 = 0.0285;
L4 = 0.11;
L5 = 0.11;

%% Conclusion:
% (1) Tp 只能改變步長，不能改變速度
% (2) Tp調大，步長變長；反之亦然
% (3) c改變腳步速度
% (4) 如果要擬真，c要調大,Tp要調小
%% Cartesian
S = c*v; % one step gait
disp(S);
% S = S*Tp/4;
Sx = S*Tp/4;
disp(Sx); %% real one step gait/2
% hf = hf*Tp/4;
Sz = hf*Tp/4;
disp(Sz); %% real one step gait
thq_r_p = linspace(pi,-pi,len);
% % cubic spline
% th_r_p = [pi pi/2 0 -pi/2 -pi];
% x_r_p = [-S 0 S 0 -S];
% thq_r_p = linspace(pi,-pi,len);
% xq_r_p = spline(th_r_p,x_r_p,thq_r_p);
% vxq_r_p = diff(xq_r_p)/dt;

vx_r_b = zeros(1,len);
for i = 1:len
    if thq_r_p(i) >= 0
        vx_r_b(i) = S/2*(1+sin(-2*(thq_r_p(i)-3*pi/4)));
    else 
        vx_r_b(i) = -S/2*(1+sin(-2*(thq_r_p(i)-3*pi/4)));
    end
end
ttp = linspace(0,2*Tp,len);
figure;
plot(ttp,vx_r_b)

xq_r_p = -Sx*ones(1,len);
% xq_r_p = zeros(1,len);
for i = 1:len-1
    xq_r_p(i+1) = xq_r_p(i) + vx_r_b(i+1)*dt;
end


ax_r_p = zeros(1,len);
ax_r_p(2:end) = diff(vx_r_b)/dt;



figure;
plot(ttp,xq_r_p)
figure;
plot(ttp,ax_r_p)

%% left 

thq_l_p = linspace(2*pi,0,len);
% % cubic spline
% th_r_p = [pi pi/2 0 -pi/2 -pi];
% x_r_p = [-S 0 S 0 -S];
% thq_r_p = linspace(pi,-pi,len);
% xq_r_p = spline(th_r_p,x_r_p,thq_r_p);
% vxq_r_p = diff(xq_r_p)/dt;


vx_l_b = zeros(1,len);
for i = 1:len
    if thq_l_p(i) >= pi
        vx_l_b(i) = -S/2*(1+sin(-2*(thq_l_p(i)-3*pi/4)));
    else 
        vx_l_b(i) = S/2*(1+sin(-2*(thq_l_p(i)-3*pi/4)));
    end
end
ttp = linspace(0,2*Tp,len);
figure;
plot(ttp,vx_l_b)

xq_l_p = Sx*ones(1,len);
% xq_l_p = zeros(1,len);
for i = 1:len-1
    xq_l_p(i+1) = xq_l_p(i) + vx_l_b(i+1)*dt;
end


ax_l_p = zeros(1,len);
ax_l_p(2:end) = diff(vx_l_b)/dt;


figure;
plot(ttp,xq_l_p)
figure;
plot(ttp,ax_l_p)


%% Try: polyfit 
%%%%%%%%%% polyfit %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 在区间 [0,4*pi] 中沿正弦曲线生成 10 个等间距的点。
% 
% x = linspace(0,4*pi,10);
% y = sin(x);
% 使用 polyfit 将一个 7 次多项式与这些点拟合。
% 
% p = polyfit(x,y,7);
% 在更精细的网格上计算多项式并绘制结果图。
% 
% x1 = linspace(0,4*pi);
% y1 = polyval(p,x1);
% figure
% plot(x,y,'o')
% hold on
% plot(x1,y1)
% hold off
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% thq_r_p1 = linspace(0,Tp,len/2);
% z_r_p1 = [0 hf/2 hf hf/2 0];
% th_r_p1 = [0 Tp/4 Tp/2 Tp/4*3 Tp];
% p = polyfit(th_r_p1,z_r_p1,4);
% zq_r_p1 = polyval(p,thq_r_p1);
% % zq_r_p1 = spline(th_r_p1,z_r_p1,thq_r_p1);
% figure;
% plot(thq_r_p1,zq_r_p1,th_r_p1,z_r_p1,'o')
%% Cartesian
% zq_r_p = zeros(1,len);
% for i = 1:length(thq_r_p)
%     if thq_r_p(i) >= 0
%         zq_r_p(i) = hf/2*(1+sin(2*thq_r_p(i)-pi/2));
%     else
%         zq_r_p(i) = 0;
%     end
% end
%% Try: design from vz --> OKOKOK!!!!!!!!!!! YAYAYAYAYAYAYAYAYA!!!!!!!!!!
vz_r_b = zeros(1,len);
for i = 1:len
    if thq_r_p(i) >= pi/2
        vz_r_b(i) = hf/2*(1+sin(-4*(thq_r_p(i)-7*pi/8)));
    elseif thq_r_p(i) >= 0
        vz_r_b(i) = -hf/2*(1+sin(-4*(thq_r_p(i)-7*pi/8)));
    else
        vz_r_b(i) = 0;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% df(x)/dx = [f(x+h) - f(x)]/h
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ttp = linspace(0,2*Tp,len);
figure;
plot(ttp,vz_r_b)

zq_r_p = zeros(1,len);
for i = 1:len-1
    zq_r_p(i+1) = zq_r_p(i) + vz_r_b(i+1)*dt;
end

%% left 
vz_l_b = zeros(1,len);
for i = 1:len
    if thq_l_p(i) >= pi
        vz_l_b(i) = 0;
    elseif thq_l_p(i) >= pi/2
        vz_l_b(i) = hf/2*(1+sin(-4*(thq_l_p(i)-7*pi/8)));
    else
        vz_l_b(i) = -hf/2*(1+sin(-4*(thq_l_p(i)-7*pi/8))); 
    end
end
ttp = linspace(0,2*Tp,len);
figure;
plot(ttp,vz_l_b)

zq_l_p = zeros(1,len);
for i = 1:len-1
    zq_l_p(i+1) = zq_l_p(i) + vz_l_b(i+1)*dt;
    if abs(zq_l_p(i+1))<1e-12
        zq_l_p(i+1) = 0;
    end
end
%% figure


figure;
plot(ttp,zq_r_p)
figure;
plot(ttp,zq_l_p)
az_r_p = zeros(1,len);
az_r_p(2:end) = diff(vz_r_b)/dt;

figure;
plot(ttp,az_r_p)

figure;
comet(xq_r_p,zq_r_p)
figure;
comet(xq_l_p,zq_l_p)
%%
% 座標轉換
eul_0 = [-pi/2 0 0]; 
rotmZYX_0 = eul2rotm(eul_0);
ptrans_0 = [0;0;-0.21];
Tm_0 = [[rotmZYX_0 ptrans_0]; 0 0 0 1];

eul_b = [0 0 0]; 
rotmZYX_b = eul2rotm(eul_b);
ptrans_b_r = [0;-L1;-L2-0.21];
Tm_b_r = [[rotmZYX_b ptrans_b_r]; 0 0 0 1];

trans_r_p = [xq_r_p;zeros(1,len);zq_r_p;ones(1,len)];
trans_r_0 = Tm_0*trans_r_p;
trans_r_b = Tm_b_r*trans_r_p;


ax_r_b = zeros(1,len);
az_r_b = zeros(1,len);

%%% a %%%
ax_r_b(2:end) = diff(vx_r_b)/dt;
az_r_b(2:end) = diff(vz_r_b)/dt;

%% Joint
gait = zeros(10,len);

% joint space 的直接算
for i = 1:len
Tm_r_p = [[eye(3); 0 0 0] [xq_r_p(i);0;zq_r_p(i);1]];
Tm_r_0 = Tm_0*Tm_r_p;
qr = leg_IK(Tm_r_0,L3,L4,L5);
q2 = qr(1); q4 = qr(2); q6 = qr(3); q8 = qr(4);
gait(:,i) =  [xq_r_p(i);vx_r_b(i);ax_r_b(i);zq_r_p(i);vz_r_b(i);az_r_b(i);...
        q2;q4;q6;q8];
% disp(i);
end

dq8 = zeros(1,len);
dq8(2:end) = diff(gait(10,:))/dt;
ddq8 = zeros(1,len);
ddq8(2:end) = diff(dq8)/dt;
%% figure
for i = 1:10
figure;
plot(tp,gait(i,:))
end    
figure;
plot(tp,dq8)

figure;
plot(tp,ddq8)












