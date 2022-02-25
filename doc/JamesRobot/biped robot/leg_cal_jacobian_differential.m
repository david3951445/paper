clc;clear;
% Example:
% q1(t) = symfun(str2sym('q1(t)'), t);
% q2(t) = symfun(str2sym('q2(t)'), t);
% q3(t) = symfun(str2sym('q3(t)'), t);
% q4(t) = symfun(str2sym('q4(t)'), t);
% 
% J11 = -sin(q1(t))*(a3*cos(q2(t) + q3(t)) + a2*cos(q2(t)))
% 
% dJ11dt = diff(J11,t)
syms th1(t) th2(t) th3(t) th4(t) th5(t) th6(t) th7(t) th8(t) t L1 L2 L3 L4 L5;
th1 = symfun(th1(t), t);
th2 = symfun(th2(t), t);
th3 = symfun(th3(t), t);
th4 = symfun(th4(t), t);
th5 = symfun(th5(t), t);
th6 = symfun(th6(t), t);
th7 = symfun(th7(t), t);
th8 = symfun(th8(t), t);
%%% Jb %%%
Jb = [                                                                                                                                                      0,   sin(th2)*(L4*sin(th6) + L5*cos(th6)*sin(th8) + L5*cos(th8)*sin(th6)) - cos(th2)*sin(th4)*(L4*cos(th6) + L5*cos(th6)*cos(th8) - L5*sin(th6)*sin(th8)),                                                                              0, -cos(th4)*sin(th2)*(L4*cos(th6) + L5*cos(th6)*cos(th8) - L5*sin(th6)*sin(th8)),                                                                                                                                                      0,   sin(th2)*sin(th4)*(L4*sin(th6) + L5*cos(th6)*sin(th8) + L5*cos(th8)*sin(th6)) - cos(th2)*(L4*cos(th6) + L5*cos(th6)*cos(th8) - L5*sin(th6)*sin(th8)),                                                                                                                          0,   sin(th2)*sin(th4)*(L5*cos(th6)*sin(th8) + L5*cos(th8)*sin(th6)) - cos(th2)*(L5*cos(th6)*cos(th8) - L5*sin(th6)*sin(th8));
                                                                                                                                                            0, - cos(th2)*(L4*sin(th6) + L5*cos(th6)*sin(th8) + L5*cos(th8)*sin(th6)) - sin(th2)*sin(th4)*(L4*cos(th6) + L5*cos(th6)*cos(th8) - L5*sin(th6)*sin(th8)),                                                                              0,  cos(th2)*cos(th4)*(L4*cos(th6) + L5*cos(th6)*cos(th8) - L5*sin(th6)*sin(th8)),                                                                                                                                                      0, - sin(th2)*(L4*cos(th6) + L5*cos(th6)*cos(th8) - L5*sin(th6)*sin(th8)) - cos(th2)*sin(th4)*(L4*sin(th6) + L5*cos(th6)*sin(th8) + L5*cos(th8)*sin(th6)),                                                                                                                          0, - sin(th2)*(L5*cos(th6)*cos(th8) - L5*sin(th6)*sin(th8)) - cos(th2)*sin(th4)*(L5*cos(th6)*sin(th8) + L5*cos(th8)*sin(th6));
                                                                                                                                                            0,                                                                                                                                                      0,                                                                              0,           sin(th4)*(L4*cos(th6) + L5*cos(th6)*cos(th8) - L5*sin(th6)*sin(th8)),                                                                                                                                                      0,                                                                                   cos(th4)*(L4*sin(th6) + L5*cos(th6)*sin(th8) + L5*cos(th8)*sin(th6)),                                                                                                                          0,                                                                     cos(th4)*(L5*cos(th6)*sin(th8) + L5*cos(th8)*sin(th6));
                                                                                                                                                            0,                                                                                                                                                      1,                                                                              0,                                                                              0,                                                                                                                                                      0,                                                                                                                                               sin(th4),                                                                                                                          0,                                                                                                                   sin(th4);
         sin(th1)*(L4*sin(th5) + L5*cos(th5)*sin(th7) + L5*cos(th7)*sin(th5)) - cos(th1)*sin(th3)*(L4*cos(th5) + L5*cos(th5)*cos(th7) - L5*sin(th5)*sin(th7)),                                                                                                                                                      0, -cos(th3)*sin(th1)*(L4*cos(th5) + L5*cos(th5)*cos(th7) - L5*sin(th5)*sin(th7)),                                                                              0,   sin(th1)*sin(th3)*(L4*sin(th5) + L5*cos(th5)*sin(th7) + L5*cos(th7)*sin(th5)) - cos(th1)*(L4*cos(th5) + L5*cos(th5)*cos(th7) - L5*sin(th5)*sin(th7)),                                                                                                                                                      0,   sin(th1)*sin(th3)*(L5*cos(th5)*sin(th7) + L5*cos(th7)*sin(th5)) - cos(th1)*(L5*cos(th5)*cos(th7) - L5*sin(th5)*sin(th7)),                                                                                                                          0;
       - cos(th1)*(L4*sin(th5) + L5*cos(th5)*sin(th7) + L5*cos(th7)*sin(th5)) - sin(th1)*sin(th3)*(L4*cos(th5) + L5*cos(th5)*cos(th7) - L5*sin(th5)*sin(th7)),                                                                                                                                                      0,  cos(th1)*cos(th3)*(L4*cos(th5) + L5*cos(th5)*cos(th7) - L5*sin(th5)*sin(th7)),                                                                              0, - sin(th1)*(L4*cos(th5) + L5*cos(th5)*cos(th7) - L5*sin(th5)*sin(th7)) - cos(th1)*sin(th3)*(L4*sin(th5) + L5*cos(th5)*sin(th7) + L5*cos(th7)*sin(th5)),                                                                                                                                                      0, - sin(th1)*(L5*cos(th5)*cos(th7) - L5*sin(th5)*sin(th7)) - cos(th1)*sin(th3)*(L5*cos(th5)*sin(th7) + L5*cos(th7)*sin(th5)),                                                                                                                          0;
                                                                                                                                                            0,                                                                                                                                                      0,           sin(th3)*(L4*cos(th5) + L5*cos(th5)*cos(th7) - L5*sin(th5)*sin(th7)),                                                                              0,                                                                                   cos(th3)*(L4*sin(th5) + L5*cos(th5)*sin(th7) + L5*cos(th7)*sin(th5)),                                                                                                                                                      0,                                                                     cos(th3)*(L5*cos(th5)*sin(th7) + L5*cos(th7)*sin(th5)),                                                                                                                          0;
                                                                                                                                                            1,                                                                                                                                                      0,                                                                              0,                                                                              0,                                                                                                                                               sin(th3),                                                                                                                                                      0,                                                                                                                   sin(th3),                                                                                                                          0];
% %%% method1 %%%
% %%% dJvdt %%%
% dJ11dt = diff(J11,t);
% dJ21dt = diff(J21,t);
% dJ31dt = diff(J31,t);
% dJ12dt = diff(J12,t);
% dJ22dt = diff(J22,t);
% dJ32dt = diff(J32,t);
% dJ13dt = diff(J13,t);
% dJ23dt = diff(J23,t);
% dJ33dt = diff(J33,t);
% dJ14dt = diff(J14,t);
% dJ24dt = diff(J24,t);
% dJ34dt = diff(J34,t);
% %%% dJwdt %%%
% dwb0dt = diff(wb0,t);
% dwb1dt = diff(wb1,t);
% dwb2dt = diff(wb2,t);
% dwb3dt = diff(wb3,t);

%%% method2  %%%
dJbdt = diff(Jb,t);
save dJbdt.mat