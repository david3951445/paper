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
    K(1,8,i) = 0.3*K(1,8,i);
    K(1,9,i) = 2*K(1,9,i);
end
rbar = [pxr;pyr;phir;vr;wr;ar;afar];
%%

xbar(1:7,1) = rbar(:,1);

eX = -0.1;
eY = -0.1;
ePhi = 0.05;
eV = 0;
eW = 0;

xbar(8:12,1) = T(xbar(3))*[eX;eY;ePhi;eV;eW];

Kbar = zeros(2,12,length(tr));% Runge Kutta
AA = zeros(12,12,length(tr));
e = zeros(5,length(tr));

e(:,1) =  [eX;eY;ePhi;eV;eW];


for i = 1:length(tr)-1
    
    Phi = xbar(10,i);
    CosPhi = cos(Phi);
    SinPhi = sin(Phi);
    Wr = xbar(5,i);
    
    xmf = [CosPhi,SinPhi,Wr];
    H = mf(xmf,xS,LP1.U);
    Kbar(:,:,i) = defuzzy(H,K);
    AA(:,:,i) = defuzzy(H,Abar);
    
    k1 = RK(xbar(:,i),rbar(:,2*i-1),go,Kbar(:,:,i),AA(:,:,i),Dbar);
    k2 = RK(xbar(:,i)+h*k1/2,rbar(:,2*i),go,Kbar(:,:,i),AA(:,:,i),Dbar);
    k3 = RK(xbar(:,i)+h*k2/2,rbar(:,2*i),go,Kbar(:,:,i),AA(:,:,i),Dbar);
    k4 = RK(xbar(:,i)+h*k3,rbar(:,2*i+1),go,Kbar(:,:,i),AA(:,:,i),Dbar);
    xbar(:,i+1) = xbar(:,i) + 1/6*h*(k1+2*k2+2*k3+k4);
    
    e(:,i+1) =  invT(xbar(3,i+1))*xbar(8:12,i+1);
    disp(i);
    
end

%% verify 
close all
% for i = 1:6
%     figure;
%     plot(tr,xbar(i,:))
% end
figure;
plot(tr,xbar(1,:),tr,e(1,:)+xbar(1,:));
figure;
plot(tr,xbar(2,:),tr,e(2,:)+xbar(2,:));
figure;
plot(tr,xbar(3,:),tr,e(3,:)+xbar(3,:));
figure;
plot(tr,xbar(4,:),tr,e(4,:)+xbar(4,:));
figure;
plot(tr,xbar(5,:),tr,e(5,:)+xbar(5,:));
figure;
plot(xbar(1,:),xbar(2,:),e(1,:)+xbar(1,:),e(2,:)+xbar(2,:));

function invT = invT(phi)
invT = [...
    cos(phi), -sin(phi), 0, 0, 0;
    sin(phi),  cos(phi), 0, 0, 0;
           0,         0, 1, 0, 0;
           0,         0, 0, 1, 0;
           0,         0, 0, 0, 1
    ];
end

function T = T(phi)
T = [...
    cos(phi),  sin(phi), 0, 0, 0;
   -sin(phi),  cos(phi), 0, 0, 0;
           0,         0, 1, 0, 0;
           0,         0, 0, 1, 0;
           0,         0, 0, 0, 1
    ];
end

function dx = RK(x,r,B,K,A,D)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 測試用：
beta = 10;
Ar = -beta*eye(7);
Br = beta*eye(7);

S2 = [zeros(2,5),eye(2)];
S3 = [zeros(2,3),eye(2),zeros(2)];


Vr = x(4);
Wr = x(5);
ex = x(8);
ey = x(9);
ePhi = x(10);
e1 = x(11);
e2 = x(12);

dx(1:7,1) = Ar*x(1:7) + Br*r;
dx(8,1) = Wr*ey + (e1+Vr)*cos(ePhi) - Vr;
dx(9,1) = -Wr*ex + (e1+Vr)*sin(ePhi);
dx(10,1) = e2;
dx(11:12,1) = S2*x(1:7) - S3*dx(1:7);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% v = [r;zeros(5,1)];
% dx = A*x + B*K*x + D*v;
dx = dx + B*K*x;
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