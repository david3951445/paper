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

for i = 1:numRule
    K(:,1:7,i) = zeros(2,7);
    
%     K(1,8,i) = 2500*K(1,8,i);
%     K(1,9,i) = 2500*K(1,9,i);
%     K(1,10,i) = -1/60*K(1,8,i);
%     K(1,11,i) = 1*K(1,11,i);
%     K(1,12,i) = -1/30*K(1,8,i);
%     
%     K(2,8,i) = 1000*K(2,10,i);
%     K(2,9,i) = 600*K(2,10,i);  
%     K(2,10,i) = 300*K(2,10,i);
%     K(2,11,i) = -0.04*K(2,10,i);    
%     K(2,12,i) = 100*K(2,12,i);

end
% K= 0.1*K;
rbar = [pxr;pyr;phir;vr;wr;ar;afar];
%%
xbar(1:7,1) = rbar(:,1);
xbar(8,1) = 0.1;
xbar(9,1) = 0.1;
xbar(10,1) = 0.05;
xbar(11,1) = 0;
xbar(12,1) = 0;

Kbar = zeros(2,12,length(tr));% Runge Kutta
AA = zeros(12,12,length(tr));

for i = 1:length(tr)-1
    Phi = xbar(10,i) + xbar(3,i);
    xmf = [Phi];
    H = mf(xmf,xS,LP1.U);
    Kbar(:,:,i) = defuzzy(H,K);
    AA(:,:,i) = defuzzy(H,Abar);
    
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
figure;
plot(xbar(1,:),xbar(2,:),xbar(7,:)+xbar(1,:),xbar(8,:)+xbar(2,:));
function dx = RK(x,r,B,K,A,D)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 實際：
beta = 10;
Ar = -beta*eye(7);
Br = beta*eye(7);
S1 = [eye(3),zeros(3,4)];
S3 = [zeros(2,3) [1,0;0,1] zeros(2)];
S5 = [zeros(2,5) eye(2)];

Phi = x(10)+x(3);
V = x(11:12)+x(4:5);


dx(1:7,1) = Ar*x(1:7) + Br*r;
dx(8:10,1) = Jt(Phi)*V - S1*dx(1:7);
dx(11:12,1) = S5*x(1:7) - S3*dx(1:7);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fuzzy Test：
% v = [r;zeros(5,1)];
% dx = A*x + B*K*x + D*v;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dx = dx + B*100*K*x;
end

function J = Jt(phi)

J = [...
    cos(phi),0;
    sin(phi),0;
    0       ,1
    ];

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