clc;clear;

%%% m I %%%
for i = 0:12 
    eval(['syms m' num2str(i) ]);
    eval(['syms Ixx' num2str(i) ]);
    eval(['syms Iyy' num2str(i) ]);
    eval(['syms Izz' num2str(i) ]);
    eval(['syms Ixy' num2str(i) ]);
    eval(['syms Ixz' num2str(i) ]);
    eval(['syms Iyz' num2str(i) ]);
end
for i = 1:12
    eval(['I' num2str(i) ' = [Ixx' num2str(i) ' Ixy' num2str(i) ' Ixz' num2str(i) '; Ixy' num2str(i) ' Iyy' num2str(i) ' Iyz' num2str(i) '; Ixz' num2str(i) ' Iyz' num2str(i) ' Izz' num2str(i) '];']);
end

%%% M %%%
for i = 1:12
    eval(['M' num2str(i) ' = blkdiag(m' num2str(i) ',m' num2str(i) ',m' num2str(i) ',I' num2str(i) ');']);
end

%%% M(th) right %%%
syms th1 th2 th3 th4 th5 th6 L1 L2 L3 L4 L5 L6
syms x1 y1 z1 x2 y2 z2 x3 y3 z3 x4 y4 z4 x5 y5 z5 x6 y6 z6 % relative position of COM

% maybe can ignore
% syms X1(th1,th2,th3,th4,th5,th6) Y1(th1,th2,th3,th4,th5,th6) Z1(th1,th2,th3,th4,th5,th6)
% syms X2(th1,th2,th3,th4,th5,th6) Y2(th1,th2,th3,th4,th5,th6) Z2(th1,th2,th3,th4,th5,th6)
% syms X3(th1,th2,th3,th4,th5,th6) Y3(th1,th2,th3,th4,th5,th6) Z3(th1,th2,th3,th4,th5,th6)
% syms X4(th1,th2,th3,th4,th5,th6) Y4(th1,th2,th3,th4,th5,th6) Z4(th1,th2,th3,th4,th5,th6)
% syms X5(th1,th2,th3,th4,th5,th6) Y5(th1,th2,th3,th4,th5,th6) Z5(th1,th2,th3,th4,th5,th6)
% syms X6(th1,th2,th3,th4,th5,th6) Y6(th1,th2,th3,th4,th5,th6) Z6(th1,th2,th3,th4,th5,th6)
% X1 = symfun(X1(th1,th2,th3,th4,th5,th6), [th1,th2,th3,th4,th5,th6]);
% Y1 = symfun(Y1(th1,th2,th3,th4,th5,th6), [th1,th2,th3,th4,th5,th6]);
% Z1 = symfun(Z1(th1,th2,th3,th4,th5,th6), [th1,th2,th3,th4,th5,th6]);
% X2 = symfun(X2(th1,th2,th3,th4,th5,th6), [th1,th2,th3,th4,th5,th6]);
% Y2 = symfun(Y2(th1,th2,th3,th4,th5,th6), [th1,th2,th3,th4,th5,th6]);
% Z2 = symfun(Z2(th1,th2,th3,th4,th5,th6), [th1,th2,th3,th4,th5,th6]);
% X3 = symfun(X3(th1,th2,th3,th4,th5,th6), [th1,th2,th3,th4,th5,th6]);
% Y3 = symfun(Y3(th1,th2,th3,th4,th5,th6), [th1,th2,th3,th4,th5,th6]);
% Z3 = symfun(Z3(th1,th2,th3,th4,th5,th6), [th1,th2,th3,th4,th5,th6]);
% X4 = symfun(X4(th1,th2,th3,th4,th5,th6), [th1,th2,th3,th4,th5,th6]);
% Y4 = symfun(Y4(th1,th2,th3,th4,th5,th6), [th1,th2,th3,th4,th5,th6]);
% Z4 = symfun(Z4(th1,th2,th3,th4,th5,th6), [th1,th2,th3,th4,th5,th6]);
% X5 = symfun(X5(th1,th2,th3,th4,th5,th6), [th1,th2,th3,th4,th5,th6]);
% Y5 = symfun(Y5(th1,th2,th3,th4,th5,th6), [th1,th2,th3,th4,th5,th6]);
% Z5 = symfun(Z5(th1,th2,th3,th4,th5,th6), [th1,th2,th3,th4,th5,th6]);
% X6 = symfun(X6(th1,th2,th3,th4,th5,th6), [th1,th2,th3,th4,th5,th6]);
% Y6 = symfun(Y6(th1,th2,th3,th4,th5,th6), [th1,th2,th3,th4,th5,th6]);
% Z6 = symfun(Z6(th1,th2,th3,th4,th5,th6), [th1,th2,th3,th4,th5,th6]);

% X(th) Y(th) Z(th) 
% xyz{i} is relative to joint i-1 (user-defined)
X1 = -y1;
Y1 = x1 - L1;
Z1 = z1 - L2;
X2 = cos(th1)*z2 - sin(th1)*x2;
Y2 = cos(th1)*x2 - L1 + sin(th1)*z2;
Z2 = y2 - L3 - L2;
X3 = - cos(th1)*y3 - sin(th1)*(cos(th2)*z3 + sin(th2)*x3);
Y3 = cos(th1)*(cos(th2)*z3 + sin(th2)*x3) - sin(th1)*y3 - L1;
Z3 = sin(th2)*z3 - L3 - cos(th2)*x3 - L2;
X4 = - sin(th1)*(cos(th2)*z4 + sin(th2)*(L4*cos(th3) + cos(th3)*x4 - sin(th3)*y4)) - cos(th1)*(L4*sin(th3) + cos(th3)*y4 + sin(th3)*x4);
Y4 = cos(th1)*(cos(th2)*z4 + sin(th2)*(L4*cos(th3) + cos(th3)*x4 - sin(th3)*y4)) - L1 - sin(th1)*(L4*sin(th3) + cos(th3)*y4 + sin(th3)*x4);
Z4 = sin(th2)*z4 - L3 - L2 - cos(th2)*(L4*cos(th3) + cos(th3)*x4 - sin(th3)*y4);
X5 = - sin(th1)*(cos(th2)*y5 + sin(th2)*(L4*cos(th3) + cos(th3)*(L5*cos(th4) + cos(th4)*x5 + sin(th4)*z5) - sin(th3)*(L5*sin(th4) - cos(th4)*z5 + sin(th4)*x5))) - cos(th1)*(L4*sin(th3) + sin(th3)*(L5*cos(th4) + cos(th4)*x5 + sin(th4)*z5) + cos(th3)*(L5*sin(th4) - cos(th4)*z5 + sin(th4)*x5));
Y5 = cos(th1)*(cos(th2)*y5 + sin(th2)*(L4*cos(th3) + cos(th3)*(L5*cos(th4) + cos(th4)*x5 + sin(th4)*z5) - sin(th3)*(L5*sin(th4) - cos(th4)*z5 + sin(th4)*x5))) - L1 - sin(th1)*(L4*sin(th3) + sin(th3)*(L5*cos(th4) + cos(th4)*x5 + sin(th4)*z5) + cos(th3)*(L5*sin(th4) - cos(th4)*z5 + sin(th4)*x5));
Z5 = sin(th2)*y5 - L3 - L2 - cos(th2)*(L4*cos(th3) + cos(th3)*(L5*cos(th4) + cos(th4)*x5 + sin(th4)*z5) - sin(th3)*(L5*sin(th4) - cos(th4)*z5 + sin(th4)*x5));
X6 = - sin(th1)*(cos(th2)*(cos(th5)*z6 + sin(th5)*x6) + sin(th2)*(L4*cos(th3) + cos(th3)*(L5*cos(th4) - sin(th4)*y6 + cos(th4)*(cos(th5)*x6 - sin(th5)*z6)) - sin(th3)*(L5*sin(th4) + cos(th4)*y6 + sin(th4)*(cos(th5)*x6 - sin(th5)*z6)))) - cos(th1)*(L4*sin(th3) + cos(th3)*(L5*sin(th4) + cos(th4)*y6 + sin(th4)*(cos(th5)*x6 - sin(th5)*z6)) + sin(th3)*(L5*cos(th4) - sin(th4)*y6 + cos(th4)*(cos(th5)*x6 - sin(th5)*z6)));
Y6 = cos(th1)*(cos(th2)*(cos(th5)*z6 + sin(th5)*x6) + sin(th2)*(L4*cos(th3) + cos(th3)*(L5*cos(th4) - sin(th4)*y6 + cos(th4)*(cos(th5)*x6 - sin(th5)*z6)) - sin(th3)*(L5*sin(th4) + cos(th4)*y6 + sin(th4)*(cos(th5)*x6 - sin(th5)*z6)))) - L1 - sin(th1)*(L4*sin(th3) + cos(th3)*(L5*sin(th4) + cos(th4)*y6 + sin(th4)*(cos(th5)*x6 - sin(th5)*z6)) + sin(th3)*(L5*cos(th4) - sin(th4)*y6 + cos(th4)*(cos(th5)*x6 - sin(th5)*z6)));
Z6 = sin(th2)*(cos(th5)*z6 + sin(th5)*x6) - L3 - cos(th2)*(L4*cos(th3) + cos(th3)*(L5*cos(th4) - sin(th4)*y6 + cos(th4)*(cos(th5)*x6 - sin(th5)*z6)) - sin(th3)*(L5*sin(th4) + cos(th4)*y6 + sin(th4)*(cos(th5)*x6 - sin(th5)*z6))) - L2;
% w  
wb0 = [0;0;1]; %% motor1 
wb1 = [cos(th1);sin(th1);0];
wb2 = [-cos(th2)*sin(th1);cos(th1)*cos(th2);sin(th2)];
wb3 = [-cos(th2)*sin(th1);cos(th1)*cos(th2);sin(th2)];
wb4 = [cos(th1)*(cos(th3)*cos(th4)-sin(th3)*sin(th4))-sin(th1)*sin(th2)*(cos(th3)*sin(th4)+cos(th4)*sin(th3));sin(th1)*(cos(th3)*cos(th4)-sin(th3)*sin(th4))+cos(th1)*sin(th2)*(cos(th3)*sin(th4)+cos(th4)*sin(th3));-cos(th2)*(cos(th3)*sin(th4)+cos(th4)*sin(th3))];
wb5 = [cos(th1)*(cos(th3)*sin(th4)*sin(th5)+cos(th4)*sin(th3)*sin(th5))-sin(th1)*(cos(th2)*cos(th5)-sin(th2)*(cos(th3)*cos(th4)*sin(th5)-sin(th3)*sin(th4)*sin(th5)));sin(th1)*(cos(th3)*sin(th4)*sin(th5)+cos(th4)*sin(th3)*sin(th5))+cos(th1)*(cos(th2)*cos(th5)-sin(th2)*(cos(th3)*cos(th4)*sin(th5)-sin(th3)*sin(th4)*sin(th5)));cos(th5)*sin(th2)+cos(th2)*(cos(th3)*cos(th4)*sin(th5)-sin(th3)*sin(th4)*sin(th5))];
o = zeros(3,1);
% J
Jv1 = jacobian([X1,Y1,Z1],[th1,th2,th3,th4,th5,th6]);
Jw1 = [wb0 o o o o o];
J1 = [Jv1;Jw1];
J1t = transpose(J1);
Jv2 = jacobian([X2,Y2,Z2],[th1,th2,th3,th4,th5,th6]);
Jw2 = [wb0 wb1 o o o o];
J2 = [Jv2;Jw2];
J2t = transpose(J2);
Jv3 = jacobian([X3,Y3,Z3],[th1,th2,th3,th4,th5,th6]);
Jw3 = [wb0 wb1 wb2 o o o];
J3 = [Jv3;Jw3];
J3t = transpose(J3);
Jv4 = jacobian([X4,Y4,Z4],[th1,th2,th3,th4,th5,th6]);
Jw4 = [wb0 wb1 wb2 wb3 o o];
J4 = [Jv4;Jw4];
J4t = transpose(J4);
Jv5 = jacobian([X5,Y5,Z5],[th1,th2,th3,th4,th5,th6]);
Jw5 = [wb0 wb1 wb2 wb3 wb4 o];
J5 = [Jv5;Jw5];
J5t = transpose(J5);
Jv6 = jacobian([X6,Y6,Z6],[th1,th2,th3,th4,th5,th6]);
Jw6 = [wb0 wb1 wb2 wb3 wb4 wb5];
J6 = [Jv6;Jw6];
J6t = transpose(J6);
% cal M
M = zeros(6,6);
for i = 1:6
    eval(['M = M + J' num2str(i) 't*M' num2str(i) '*J' num2str(i) ';']);
end

%%% V(th) %%%
syms g % gravity
for i = 1:6
    eval(['h' num2str(i) ' = Z' num2str(i) ';']);
    eval(['V' num2str(i) ' = m' num2str(i) '*g*h' num2str(i) ';']);
end
V = 0;
for i = 1:6
    eval(['V = V + V' num2str(i) ';']); 
end

%%% N(th) %%%
N = transpose(jacobian(V,[th1,th2,th3,th4,th5,th6]));

%%% C(th) %%%
syms dth1dt dth2dt dth3dt dth4dt dth5dt dth6dt
th = [th1;th2;th3;th4;th5;th6];
dthdt = [dth1dt;dth2dt;dth3dt;dth4dt;dth5dt;dth6dt];

for i = 1:6
    for j = 1:6
        sum = 0;
        for k = 1:6
            sum = sum + (diff(M(i,j),th(k)) + diff(M(i,k),th(j)) - diff(M(k,j),th(i)))*dthdt(k);
        end
        C(i,j) = sum/2;
    end
end

%%% H(th) %%%
H = C*dthdt + N;

save dyn_para.mat