clc;clear;
load dyn_para.mat

% th --> rad 
for i = 0:12 
    eval(['m' num2str(i) ' = 1;']);
    eval(['Ixx' num2str(i) ' = 1;']);
    eval(['Iyy' num2str(i) ' = 1;']);
    eval(['Izz' num2str(i) ' = 1;']);
    eval(['Ixy' num2str(i) ' = 1;']);
    eval(['Ixz' num2str(i) ' = 1;']);
    eval(['Iyz' num2str(i) ' = 1;']);
end
x1 = 1;
y1 = 1; 
z1 = 1; 
x2 = 1;
y2 = 1; 
z2 = 1; 
x3 = 1; 
y3 = 1;
z3 = 1;
x4 = 1;
y4 = 1;
z4 = 1;
x5 = 1; 
y5 = 1; 
z5 = 1; 
x6 = 1; 
y6 = 1; 
z6 = 1;
th1 = 0.2;
th2 = 0.3;
th3 = 0.3;
th4 = 0.1;
th5 = -0.3;
th6 = -0.2;
L1 = 0.035;
L2 = 0.0907;
L3 = 0.0285;
L4 = 0.11;
L5 = 0.11;
L6 = 0.0305;
MM = subs(M);
MMM = double(MM);