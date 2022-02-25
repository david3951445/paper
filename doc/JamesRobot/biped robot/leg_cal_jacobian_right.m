clc;clear;
%% 相對於 base frame
syms th2 th4 th6 th8 xr(th2,th4,th6,th8) yr(th2,th4,th6,th8) zr(th2,th4,th6,th8) L1 L2 L3 L4 L5;
xr = symfun(xr(th2,th4,th6,th8), [th2,th4,th6,th8]);
yr = symfun(yr(th2,th4,th6,th8), [th2,th4,th6,th8]);
zr = symfun(zr(th2,th4,th6,th8), [th2,th4,th6,th8]);
%%% right leg %%%
xr = - cos(th2)*(L4*sin(th6) + L5*cos(th6)*sin(th8) + L5*cos(th8)*sin(th6)) - sin(th2)*sin(th4)*(L4*cos(th6) + L5*cos(th6)*cos(th8) - L5*sin(th6)*sin(th8));
yr = cos(th2)*sin(th4)*(L4*cos(th6) + L5*cos(th6)*cos(th8) - L5*sin(th6)*sin(th8)) - sin(th2)*(L4*sin(th6) + L5*cos(th6)*sin(th8) + L5*cos(th8)*sin(th6)) - L1;
zr = - L2 - L3 - cos(th4)*(L4*cos(th6) + L5*cos(th6)*cos(th8) - L5*sin(th6)*sin(th8));
%%% w %%% 
wb0r = [0;0;1];
wb1r = [cos(th2);sin(th2);0];
wb2r = [-cos(th4)*sin(th2);cos(th2)*cos(th4);sin(th4)];
wb3r = [-cos(th4)*sin(th2);cos(th2)*cos(th4);sin(th4)];
%%% method1 %%%
J11r = diff(xr,th2);
J21r = diff(yr,th2);
J31r = diff(zr,th2);
J12r = diff(xr,th4);
J22r = diff(yr,th4);
J32r = diff(zr,th4);
J13r = diff(xr,th6);
J23r = diff(yr,th6);
J33r = diff(zr,th6);
J14r = diff(xr,th8);
J24r = diff(yr,th8);
J34r = diff(zr,th8);
% for i = 1:3
%     for j = 1:4
%         eval(['J' num2str(i) '' num2str(i) 'r = simplify(J' num2str(i) '' num2str(i) 'r);']);
%     end
% end

%%% method2 %%%
Jvr = jacobian([xr,yr,zr],[th2,th4,th6,th8]);
Jwr = [wb0r wb1r wb2r wb3r];
Jr = [Jvr;Jwr];
Jbr = cat(1,Jr(1:3,:),Jr(6,:)); %% what we care is xr,yr,zr & phi
Jbr = simplify(Jbr);
save Jbr.mat