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

LP1.P = size(LP1.domain,1);
numRule = prod(size(LP1.S,1:LP1.P));

% 
% for i = 1:3
%     K(:,1:7,i) = zeros(2,7);
%     tmp = K(:,8,i);
%     K(:,8,i) = 2*K(:,11,i);
%     K(:,9,i) = K(:,11,i);
%     K(:,12,i) = 6*K(:,12,i);
% end
% K = 5*K;
for i = 1:numRule
    K(:,1:7,i) = zeros(2,7);
    
    % %     for j = 1:2
    % %         for k = 8:12
    % %             if K(j,k,i) > 0
    % %                 K(j,k,i) = -K(j,k,i);
    % %             end
    % %         end
    % %     end
    % %     K(:,12,i) = 1.7*K(:,12,i);
    
    K(1,8,i) = 100*K(1,8,i);
    K(1,9,i) = 100*K(1,9,i);
    K(1,10,i) = 10*K(1,10,i);
    K(1,11,i) = 0.1*K(1,11,i);
    K(1,12,i) = 10*K(1,12,i);
% % % %     
    K(2,8,i) = 100*K(2,8,i);
    K(2,9,i) = 100*K(2,9,i);  
    K(2,10,i) = 0.5*K(2,10,i);
    K(2,11,i) = 10*K(2,11,i);    
    K(2,12,i) = 1*K(2,12,i);

end
rbar = [pxr;pyr;phir;vr;wr;ar;afar];
%%
xbar(1:7,1) = rbar(:,1);
xbar(8,1) = 0.1;
xbar(9,1) = 0.1;
xbar(10,1) = 0.1;
xbar(11,1) = 0;
xbar(12,1) = 0;

Kbar = zeros(2,12,length(tr));% Runge Kutta
AA = zeros(12,12,length(tr));

for i = 1:length(tr)-1
    Phi = xbar(10,i) + xbar(3,i);
    V = xbar(11,i) + xbar(4,i);
    W = xbar(12,i) + xbar(5,i);
    xmf = [Phi,V,W];
    H = mf(xmf,xS,LP1.U);
    Kbar(:,:,i) = defuzzy(H,K);
    AA(:,:,i) = defuzzy(H,Abar);
    
    EIG(:,i) = eig(AA(:,:,i)+go*Kbar(:,:,i));
    
    k1 = RK(xbar(:,i),rbar(:,2*i-1),go,Kbar(:,:,i),AA(:,:,i),Dbar);
    k2 = RK(xbar(:,i)+h*k1/2,rbar(:,2*i),go,Kbar(:,:,i),AA(:,:,i),Dbar);
    k3 = RK(xbar(:,i)+h*k2/2,rbar(:,2*i),go,Kbar(:,:,i),AA(:,:,i),Dbar);
    k4 = RK(xbar(:,i)+h*k3,rbar(:,2*i+1),go,Kbar(:,:,i),AA(:,:,i),Dbar);
    xbar(:,i+1) = xbar(:,i) + 1/6*h*(k1+2*k2+2*k3+k4);
    disp(i);
end

%% verify 
close all
% for i = 1:6
%     figure;
%     plot(tr,xbar(i,:))
% end
figure;
plot(tr,xbar(1,:),tr,xbar(8,:)+xbar(1,:));
figure;
plot(tr,xbar(2,:),tr,xbar(9,:)+xbar(2,:));
figure;
plot(tr,xbar(3,:),tr,xbar(10,:)+xbar(3,:));
figure;
plot(tr,xbar(4,:),tr,xbar(11,:)+xbar(4,:));
figure;
plot(tr,xbar(5,:),tr,xbar(12,:)+xbar(5,:));

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
v = [r;zeros(5,1)];
dx = A*x + B*K*x + D*v;

% dx = A*x + D*v;
end

function Kbar = defuzzy(H,K)
c = 1;
Kbar = 0; 
for i1 = 1:size(H{1},2)
    for i2 = 1:size(H{2},2)
        for i3 = 1:size(H{3},2)
            h(c) = H{1}(i1)*H{2}(i2)*H{3}(i3);
            c = c+1;
        end
    end
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