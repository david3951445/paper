%% 
clc;clear;
S1 = [eye(8) zeros(8,4)];
S2 = [zeros(1,2) eye(1)];
S3 = [eye(4) zeros(4,11)];
S5 = ones(2)-eye(2);
S4 = [zeros(2,8) S5 zeros(2,5)];
S6 = [S1' [zeros(4,2) eye(4) [zeros(2) eye(2)]' eye(4)]']';
S7 = [eye(3) zeros(3,1)];
S9 = [S7 zeros(3,2)];
S8 = [zeros(6,12) S9' zeros(6,12)]';
S13 = [zeros(4,2) -eye(4) [zeros(2) -eye(2)]']';
S10 = blkdiag(eye(8),S13)';
S11 = [eye(4) zeros(4,1)];
S12 = [eye(3) zeros(3,2)];

d = 0.2;
beta = 10;
Ar = -beta*eye(15);
%%
load('G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\tractor-trailer\dJtdt.mat');
load('G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\tractor-trailer\Jt.mat');

J_bar = pinvJt*dJtdt*pinvJt*S11;
pinvJt = pinvJt*S11;
J_bar = simplify(J_bar);
J_bar = subs(J_bar, {'phi0', 'phi1', 'dphi0dt', 'dphi1dt'},...
    {str2sym('x(1)'), str2sym('x(2)'), str2sym('x(3)'), str2sym('x(4)')});
pinvJt = subs(pinvJt, {'phi0', 'phi1', 'dphi0dt', 'dphi1dt'},...
    {str2sym('x(1)'), str2sym('x(2)'), str2sym('x(3)'), str2sym('x(4)')});

% %%
% load('G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot\Je.mat');
% Je = simplify(Je);
% Je = subs(Je, {'th1','th2','th3','th4','th5','th6','th7','th8'},...
%     {str2sym('x(5)'), str2sym('x(6)'), str2sym('x(7)'), str2sym('x(8)'),...
%     str2sym('x(9)'), str2sym('x(10)'), str2sym('x(11)'), str2sym('x(12)')});
% JeS = simplify(S4*Je);

%%
load('G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\tractor-trailer\Jt.mat');
JtS = simplify(S7*Jt);
JtS = subs(JtS, {'phi0', 'phi1'}, {str2sym('x(1)'), str2sym('x(2)')});

%%
load('G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot\G.mat');
GS = simplify(G*S1);
GS = subs(GS, {'th'}, {str2sym('x(5)')});

%%
load('G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot\G.mat');
G_bar1 = simplify(dpinvGdt*S7);
G_bar1 = subs(G_bar1, {'th', 'dthdt'}, {str2sym('x(10)'), str2sym('x(11)')});
G_bar2 = simplify(G_pinv*S7);
G_bar2 = subs(G_bar2, {'th', 'dthdt'}, {str2sym('x(10)'), str2sym('x(11)')});
