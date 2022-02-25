clc;clear;
%% 相對於 base frame 
syms th1 th3 th5 th7 xl(th1,th3,th5,th7) yl(th1,th3,th5,th7) zl(th1,th3,th5,th7) L1 L2 L3 L4 L5;
xl = symfun(xl(th1,th3,th5,th7), [th1,th3,th5,th7]);
yl = symfun(yl(th1,th3,th5,th7), [th1,th3,th5,th7]);
zl = symfun(zl(th1,th3,th5,th7), [th1,th3,th5,th7]);
%%% left leg %%%
xl = - cos(th1)*(L4*sin(th5) + L5*cos(th5)*sin(th7) + L5*cos(th7)*sin(th5)) - sin(th1)*sin(th3)*(L4*cos(th5) + L5*cos(th5)*cos(th7) - L5*sin(th5)*sin(th7));
yl = cos(th1)*sin(th3)*(L4*cos(th5) + L5*cos(th5)*cos(th7) - L5*sin(th5)*sin(th7)) - sin(th1)*(L4*sin(th5) + L5*cos(th5)*sin(th7) + L5*cos(th7)*sin(th5)) + L1;
zl = - L2 - L3 - cos(th3)*(L4*cos(th5) + L5*cos(th5)*cos(th7) - L5*sin(th5)*sin(th7));
%%% w %%% 
wb0l = [0;0;1];
wb1l = [cos(th1);sin(th1);0];
wb2l = [-cos(th3)*sin(th1);cos(th1)*cos(th3);sin(th3)];
wb3l = [-cos(th3)*sin(th1);cos(th1)*cos(th3);sin(th3)];
%%% method1 %%%
J11l = diff(xl,th1);
J21l = diff(yl,th1);
J31l = diff(zl,th1);
J12l = diff(xl,th3);
J22l = diff(yl,th3);
J32l = diff(zl,th3);
J13l = diff(xl,th5);
J23l = diff(yl,th5);
J33l = diff(zl,th5);
J14l = diff(xl,th7);
J24l = diff(yl,th7);
J34l = diff(zl,th7);
% for i = 1:3
%     for j = 1:4
%         eval(['J' num2str(i) '' num2str(i) 'l = simplify(J' num2str(i) '' num2str(i) 'l);']);
%     end
% end
% 
% Jvll = [J11l J12l J13l J14l;
%         J21l J22l J23l J24l;
%         J31l J32l J33l J34l];

%%% method2 %%%
Jvl = jacobian([xl,yl,zl],[th1,th3,th5,th7]);
Jwl = [wb0l wb1l wb2l wb3l];
Jl = [Jvl;Jwl];
Jbl = cat(1,Jl(1:3,:),Jl(6,:)); %% what we care is xl,yl,zl & phi
Jbl = simplify(Jbl);
save Jbl.mat