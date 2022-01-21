clc;clear;close all
load 'G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot\com0_trajectory_reference_model.mat';
%%
% parameter
d = 0.2; % link bar length: 20cm
% initialization
phi0 = zeros(1,length(tr));
x0 = zeros(1,length(tr));
y0 = zeros(1,length(tr));

phi0(1) = xr(7,1);
x0(1) = xr(1,1) + d*cos(xr(7,1));
y0(1) = xr(2,1) + d*sin(xr(7,1));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1. holonomic:
%       x0 = x1 + d*cos(phi1)
%       y0 = y1 + d*sin(phi1)
% Conclusion:
%   (1) may be error accumulated, phi0 does not make sense in the
%   beginning.
% 2. nonholonomic: recursive
%        vx1 = vx0*cos(phi0-phi1)
%        vy1 = vy0*cos(phi0-phi1)
%        w1  = v0*sin(phi0-phi1)/d   or   use diff(.)
% Conclusion:
%   (1) If w is calculate by noholonomic constraints, it will appear
%   singular value.
%   (2)Therefore, we use atan(.) to calculate phi0 and use diff(.) to
%   calculate w0.
%
% 3. Compare & Result:
%   (1) Velocity: 
%       The velocity which is calculated from holonomic constraints is same
%       as the one which is calculated from nonholonomic constraints.
%   (2) Angular:
%       i.   Equation: phi0 = phi1 + atan(d*w1/v1).
%       ii.  use equation instead of atan(vy/vx).
%       iii. the variation of phi0(w0,afa0) is greater than phi1(w1,afa1)
%   (3) Position:
%       The result from nonholonomic constraint is same as from holonomic
%       constraint.
%   (4) Result:
%       phi0 is calculated from nonholonomic constraint, others can use
%       both constraints. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
v0 = zeros(1,length(tr));
vx0 = zeros(1,length(tr));
vy0 = zeros(1,length(tr));
% nonholonomic
for i = 1:length(tr)
    if i ~= 1
        v0(i) = v(i)/cos(phi0(i)-xr(7,i)); % O
        vx0(i) = v0(i)*cos(phi0(i)); % O
        vy0(i) = v0(i)*sin(phi0(i)); % O
        x0(i) = x0(i-1) + vx0(i)*hr; % O
        y0(i) = y0(i-1) + vy0(i)*hr; % O
    end
    if i ~= length(tr)
        phi0(i+1) = xr(7,i+1) + atan(d*xr(8,i+1)/v(i+1)); % O
    end
end
ax0 = diff(vx0)/hr; ax0 = [0 ax0];
ay0 = diff(vy0)/hr; ay0 = [0 ay0];
w0 = diff(phi0)/hr; w0 = [0 w0];
afa0 = diff(w0)/hr; afa0 = [0 afa0];
% holonomic
x0_test = zeros(1,length(tr));
y0_test = zeros(1,length(tr));
for i = 1:length(tr)
    x0_test(i) = xr(1,i) + d*cos(xr(7,i)); % X
    y0_test(i) = xr(2,i) + d*sin(xr(7,i)); % X
end
vx0_test = diff(x0_test)/hr; vx0_test = [0 vx0_test]; % X
vy0_test = diff(y0_test)/hr; vy0_test = [0 vy0_test]; % X
ax0_test = diff(vx0_test)/hr; ax0_test = [0 ax0_test]; % X
ay0_test = diff(vy0_test)/hr; ay0_test = [0 ay0_test]; % X

phi0_test = atan(vy0_test./vx0_test); % X (In the beginning, it dose not make snese )

v0_test = (vx0_test.^2 + vy0_test.^2).^(1/2); % O

phi1_test = zeros(1,length(tr));
for i = 1:length(tr)
    phi1_test(i) = 1/d*v(i)*tan(phi0(i)-xr(7,i));
end

figure;
plot(tr,v0,tr,v,'--',tr,v0_test,'g'); % 1
figure;
plot(tr,phi0,tr,xr(7,:),'--',tr,phi0_test,'g'); % 1
figure;
plot(tr,vx0,tr,xr(3,:),'--',tr,vx0_test,'g'); % 1
figure;
plot(tr,vy0,tr,xr(4,:),'--',tr,vy0_test,'g'); % 1
figure;
plot(tr,ax0,tr,xr(5,:),'--',tr,ax0_test,'g'); % 1
figure;
plot(tr,ay0,tr,xr(6,:),'--',tr,ay0_test,'g'); % 1

figure;
plot(tr,x0,tr,xr(1,:),'--',tr,x0_test,'g'); % 1
figure;
plot(tr,y0,tr,xr(2,:),'--',tr,y0_test,'g'); % 1
figure;
plot(x0,y0,xr(1,:),xr(2,:),'--',x0_test,y0_test,'g'); % 1

figure;
plot(tr,w0,tr,xr(8,:),'--'); % 1
figure;
plot(tr,afa0,tr,xr(9,:),'--'); % 1
figure;
plot(tr,xr(8,:),'--',tr,phi1_test,'g'); 
%%% show 2 animated line at the same time %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Result: Can not use, since it is too slow.
% 
% ex.
% clear;clc;close all
% h1 = animatedline;
% h2 = animatedline;
% axis([0,4*pi,-1,1])
% 
% x = linspace(0,4*pi,1000);
% y1 = sin(x);
% y2 = -sin(x);
% for k = 1:length(x)
%     addpoints(h1,x(k),y1(k));
%     addpoints(h2,x(k),y2(k));
%     drawnow
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


