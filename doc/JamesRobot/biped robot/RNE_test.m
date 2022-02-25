clc;clear; 
%% moving frames algorithm 
syms q1 q2 l1 l2 dq1 dq2 ddq1 ddq2 g m1 m2
A1 = [
    cos(q1),-sin(q1),0,l1*cos(q1);
    sin(q1),cos(q1),0,l1*sin(q1);
    0,0,1,0;
    0,0,0,1
    ];

A2 = [
    cos(q2),-sin(q2),0,l2*cos(q2);
    sin(q2),cos(q2),0,l2*sin(q2);
    0,0,1,0;
    0,0,0,1
    ];


R01 = A1(1:3,1:3);
R10 = transpose(R01);
R12 = A2(1:3,1:3);
R21 = transpose(R12);

w0 = [0;0;0];
z0 = [0;0;1];
dw0 = [0;0;0];
r01 = A1(1:3,4);
a0 = [0;g;0];
r1c1 = [0;0;0];

w1 = simplify(R10*(w0+dq1*z0));
dw1 = simplify(R10*(dw0+ddq1*z0+dq1*cross(w0,z0)));
a1 = simplify(R10*a0 + cross(dw1,R10*r01) + cross(w1,cross(w1,R10*r01))); 
ac1 = simplify(a1 + cross(dw1,r1c1) + cross(w1,cross(w1,r1c1)));


z1 = [0;0;1];
r12 = A2(1:3,4);
r2c2 = [0;0;0];

w2 = simplify(R21*(w1+dq2*z1));
dw2 = simplify(R21*(dw1+ddq2*z1+dq2*cross(w1,z1)));
a2 = simplify(R21*a1 + cross(dw2,R21*r12) + cross(w2,cross(w2,R21*r12))); 
ac2 = simplify(a2 + cross(dw2,r2c2) + cross(w2,cross(w2,r2c2)));


f3 = [0;0;0];
n3 = [0;0;0];
N2 = [0;0;0];

f2 = f3 + m2*ac2;
n2 = n3 - cross(f2,R21*(r12+r2c2)) + cross(f3,R21*r2c2) + N2;
tau2 = simplify(transpose(n2)*(R21*z1));

N1 = [0;0;0];

f1 = R12*f2 + m1*ac1;
n1 = n2 - cross(f1,R10*(r01+r1c1)) + cross(R12*f2,R10*r1c1) + N1;
tau1 = simplify(transpose(n1)*(R10*z0));