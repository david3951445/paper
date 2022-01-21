% Conclusion: 
%   (1) In method 1, dr and v are different.
%   (2) In method 2, dr and v are same.
clc;clear;close all

load('G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot\com0_trajectory_reference_model.mat');
% method 1
% test_r = (x0r.^2 + y0r.^2).^(1/2);
% test_dr = (1/2)*(x0r.^2 + y0r.^2).^(-1/2).*(2*x0r.*vx0r + 2*y0r.*vy0r);
% test_v = (vx0r.^2 + vy0r.^2).^(1/2);
% 
% figure;
% plot (t,test_r);
% figure;
% plot (t,test_dr,t,test_v);

% method 2
% r is total length
test_l = (diff(x0r).^(2) + diff(y0r).^(2)).^(1/2);

test_r(1) = 0;
for i = 1:length(t)-1
    test_r(i+1) = test_r(i) + test_l(i);
end
test_dr = diff(test_r)/hh;
test_dr = [0 test_dr];
test_v = (vx0r.^2 + vy0r.^2).^(1/2);
figure;
plot(t,test_dr-test_v);
figure;
plot(t,test_dr,'k--',t,test_v);

