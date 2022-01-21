clc;clear;close all
% x = [phi0 phi1 v1 w0]
load('G:\我的雲端硬碟\畢業論文 進度\模擬\MATLAB\biped robot\com0_trajectory_reference_model.mat');
LP1 = load('lpv_data.mat');
load ('gain_L.mat');
%%
xS_domain = [LP1.domain];
xS_gridsize = [LP1.gridsize];
xS = cell(size(xS_domain,1),1);
% sample x
for i = 1:size(xS_domain,1)
    xS{i} = linspace(xS_domain(i,1),xS_domain(i,2),xS_gridsize(i));
end
%% adjust K gain
% 
for i = 1:2
    K(:,1:6,i) = zeros(2,6);
    tmp = K(:,7,i);
    K(:,7,i) = K(:,9,i);
    K(:,9,i) = 100*tmp;
%     K(2,:,i) = 10*K(2,:,i);
end
K(1,9,1) = 10*K(1,9,1);
LP1.P = size(LP1.domain,1);
numRule = prod(size(LP1.S,1:LP1.P));

rbar = [pxr;phir;vr;wr;ar;afar];
%%
xbar(1:6,1) = rbar(:,1);
xbar(7,1) = 1;
xbar(8,1) = 0.2;
xbar(9,1) = 1;
xbar(10,1) = 0.01;

Kbar = zeros(2,10,length(tr));% Runge Kutta
AA = zeros(10,10,length(tr));


for i = 1:length(tr)-1
    Phi = xbar(8,i) + xbar(2,i);
    xmf = Phi;
    H = mf(xmf,xS,LP1.U);
    Kbar(:,:,i) = defuzzy(H,K);
    AA(:,:,i) = defuzzy(H,Abar);
    
    tu(:,:,i) = Kbar(:,:,i)*xbar(:,i);
    ta(:,:,i) = AA(:,:,i)*xbar(:,i);
    
    k1 = RK(xbar(:,i),rbar(:,2*i-1),go,Kbar(:,:,i),AA(:,:,i),Dbar);
    k2 = RK(xbar(:,i)+h*k1/2,rbar(:,2*i),go,Kbar(:,:,i),AA(:,:,i),Dbar);
    k3 = RK(xbar(:,i)+h*k2/2,rbar(:,2*i),go,Kbar(:,:,i),AA(:,:,i),Dbar);
    k4 = RK(xbar(:,i)+h*k3,rbar(:,2*i+1),go,Kbar(:,:,i),AA(:,:,i),Dbar);
    xbar(:,i+1) = xbar(:,i) + 1/6*h*(k1+2*k2+2*k3+k4);
    disp(i);
end

%% verify 
close all
for i = 1:6
    figure;
    plot(tr,xbar(i,:))
end
figure;
plot(tr,xbar(1,:),tr,xbar(7,:)+xbar(1,:));
figure;
plot(tr,xbar(2,:),tr,xbar(8,:)+xbar(2,:));
figure;
plot(tr,xbar(3,:),tr,xbar(9,:)+xbar(3,:));
figure;
plot(tr,xbar(4,:),tr,xbar(10,:)+xbar(4,:));
function dx = RK(x,r,B,K,A,D)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % 測試用：
% beta = 10;
% Ar = -beta*eye(15);
% Br = beta*eye(15);
% S3 = [eye(4) zeros(4,11)];
% S11 = [eye(4) zeros(4,1)];
% S5 = [0 1;1 0];
% S4 = [zeros(2,8) S5 zeros(2,5)];
% dx(1:15,1) = Ar*x(1:15) + Br*r;
% dx(16:19,1) = Jt(S5*x(18:19)+S5*x(3:4),d) * (x(20:21)+S5*x(9:10)) - S3*dx(1:15); % [x1 y1 phi(1) phi(2)]
% dx(20:21,1) = pinvJt(S5*x(18:19)+S5*x(3:4),d) * S11 * x(11:15) - pinvJt(S5*x(18:19)+S5*x(3:4),d) * dJtdt(x(20:21)+S5*x(8:9),S5*x(18:19)+S5*x(3:4),d) * pinvJt(S5*x(18:19)+S5*x(3:4),d) * S11 * x(6:10) - S4*dx(1:15); % [w0 w1]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
v = [r;zeros(4,1)];
dx = A*x + B*K*x + D*v;

% dx = A*x + D*v;
end

function Kbar = defuzzy(H,K)
c = 1;
Kbar = 0; 
for i1 = 1:size(H{1},2)
    h(c) = H{1}(i1);
    c = c+1;
end

for i = 1:c-1
    Kbar = h(i)*K(:,:,i) + Kbar;
end

end
function H = mf(x,xS,U)
    H = cell(1,size(U,2));
    n = zeros(1,size(U,2)); 
    for i = 1:size(U,2)
        n(i) = size(U{i},1); % number of sample point of each premise variable
    end
    
    for i = 1:size(U,2) % number of premise variable
        for j = 1:n(i)
            if x(i) <= xS{i}(1)
                H{i} = U{i}(1,:);
                break;
            elseif x(i) <= xS{i}(j)
                h1 = (x(i)-xS{i}(j-1))/(xS{i}(j)-xS{i}(j-1));
                h2 = 1-h1;
                %%% 分點公式 %%%
                H{i} = h1*U{i}(j,:) + h2*U{i}(j-1,:);
                break;
            elseif x(i) > xS{i}(n(i))
                H{i} = U{i}(n(i),:);
                break;
            end
        end
    end
end