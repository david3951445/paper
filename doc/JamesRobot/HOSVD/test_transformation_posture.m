clc;clear;close all
addpath 'G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot'
% test the correctness of transformation matrix 
L1 = 0.035;
L2 = 0.0907;
L3 = 0.0285;
L4 = 0.11;
L5 = 0.11;
L6 = 0.0305;
L7 = 0.02;

z = 0.21; % 站立時的高度 
d = 0.2485-z;


eul_0 = [-pi/2 0 0]; 
rotmZYX_0 = eul2rotm(eul_0);
ptrans_0 = [0;0;-z];
Tm_0 = [[rotmZYX_0 ptrans_0]; 0 0 0 1];

eul_b = [0 0 0]; 
rotmZYX_b = eul2rotm(eul_b);
ptrans_b_r = [0;-L1;-L2-z];
Tm_b_r = [[rotmZYX_b ptrans_b_r]; 0 0 0 1];

sample_trans_r_p = [0;0;0;1];
sample_trans_r_0 = Tm_0*sample_trans_r_p;
sample_trans_r_b = Tm_b_r*sample_trans_r_p;



Tm_r_p = [[eye(3); 0 0 0] sample_trans_r_p];
Tm_r_0 = Tm_0*Tm_r_p;
q = leg_IK(Tm_r_0,L3,L4,L5);

th = pi/6;
xb = 0;
yb = 0;
zb = z+L2+L6;
xb1 = xb;
yb1 = yb;
zb1 = zb - L2;
x0 = xb + L1*cos(th-pi/2);
y0 = yb + L1*sin(th-pi/2);
z0 = zb - L2;
x1 = x0;
y1 = y0;
z1 = z0-L3;
x2=x1+L4*cos(-pi/2-q(3))*cos(th);
y2=y1+L4*cos(-pi/2-q(3))*sin(th);
z2=z1+L4*sin(-pi/2-q(3));
x3=x2+L5*cos(-pi/2-q(3)-q(4))*cos(th);
y3=y2+L5*cos(-pi/2-q(3)-q(4))*sin(th);
z3=z2+L5*sin(-pi/2-q(3)-q(4));
x4=x3+L6*cos(-pi/2)*cos(th);
y4=y3+L6*cos(-pi/2)*sin(th);
z4=z3+L6*sin(-pi/2);
x5=x4+L7*cos(0)*cos(th);
y5=y4+L7*cos(0)*sin(th);
z5=z4+L7*sin(0);

plot3([xb, xb1, x0, x1, x2, x3, x4, x5],[yb, yb1, y0, y1, y2, y3, y4, y5],[zb, zb1, z0, z1, z2, z3, z4, z5])
grid on
axis equal
% axis([0 15 0 15 0 0.3312])
% x0 = 0;
% y0 = -d;
% x1 = 0;
% y1 = -L3-d;
% x2=x1+L4*cos(-pi/2-q(3));
% y2=y1+L4*sin(-pi/2-q(3));
% x3=x2+L5*cos(-pi/2-q(3)-q(4));
% y3=y2+L5*sin(-pi/2-q(3)-q(4));
% x4=x3+L6*cos(-pi/2);
% y4=y3+L6*sin(-pi/2);
% x5=x4+L7*cos(0);
% y5=y4+L7*sin(0);
% 
% figure;
% plot([x0, x1, x2, x3, x4, x5],[y0, y1, y2, y3, y4, y5])
% axis([-0.12 0.12 -0.2790 0])
% grid on
