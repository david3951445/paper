clc;clear;close all
%% load matrices
LP1 = load('lpv_data_part1_v5.mat');

options = sdpsettings('solver','mosek');
options = sdpsettings(options,'verbose',0);
%% part1 
LP1.P = size(LP1.domain,1);
A1 = cell(size(LP1.S,1:LP1.P));
for i = 1:size(LP1.S,1)
    for j = 1:size(LP1.S,2)
        A1{i,j}(:,:) = LP1.S(i,j,:,:);
    end
end


%% parameter
S3 = [eye(4),zeros(4)];
S4 = [zeros(2,4),eye(2),zeros(2)];
S5 = [zeros(2,6),eye(2)];

d = 0.2;
beta = 10;
Ar = -beta*eye(8);
Br = beta*eye(8);
BrS = [-S3*Br;-S4*Br];
Dbar = [...
    Br           zeros(8,6) ;
    BrS          eye(6)      ;
];
numRule = prod(size(LP1.S,1:LP1.P));
%% construct Abar
Abar = zeros(14,14,numRule);
Bbar = zeros(14,2,numRule);
c = 1;
for i1 = 1:size(LP1.S,1)
    for i2 = 1:size(LP1.S,2)
        Abar(:,:,c) = [...
            Ar                            zeros(8,4)  zeros(8,2)           ;
            A1{i1,i2}(:,1:4)'*S4 - S3*Ar  zeros(4,4)  A1{i1,i2}(:,1:4)'     ;
            S5 - S4*Ar                    zeros(2,4)  zeros(2)              ;
            ];
        
        Bbar(:,:,c) = [...
            zeros(12,2);
            A1{i1,i2}(:,5:6);
        ];
        c = c+1;
    end
end

%% weighting matices 
Q1 = [...
    1 0 0 0 ;
    0 1 0 0 ;
    0 0 1 0 ;
    0 0 0 1 ;
];
Q2 = [...
    0.5 0;
    0 0.5;
];
epsilon = 0.1;
Qbar = 1e-3 * blkdiag(zeros(8),Q1,Q2);
% QbarS = sqrtm(Qbar);
Rbar =  1e-2 * (eye(2) + epsilon^2*eye(2));
invRbar = inv(Rbar);
lo = 10;
dF = 0.001;
D2 = Dbar*Dbar';
%% Lmi
K = zeros(2,14,numRule);
W = sdpvar(14,14);
Y = sdpvar(2,14,numRule,'full');
str = 'Infeasible problem (MOSEK)';
Ct = 0;
for i = 1:numRule
    %%% 目前缺少 Abar{-i,k} %%%
%     Ct = 0;
%     W = sdpvar(18,18);
%     Y = sdpvar(2,18,'full');
    
    P11 = Abar(:,:,i)*W + W*Abar(:,:,i)' + Bbar(:,:,i)*Y(:,:,i) + Y(:,:,i)'*Bbar(:,:,i)' + 2*eye(14) + (1/lo)^2*D2;
    P21 = Y(:,:,i);
    P22 = -invRbar;
    P31 = Qbar*W;
    P32 = zeros(14,2);
    P33 = -eye(14);
    P41 = dF*W;
    P42 = zeros(14,2);
    P43 = zeros(14);
    P44 = -eye(14);


    LMI = [ P11 P21' P31' P41';...
            P21 P22  P32' P42';...
            P31 P32  P33  P43';...
            P41 P42  P43  P44];

    Ct = [Ct, LMI<=0];
%     Ct = [Ct, W>=0];
    disp(i)
%     sol = optimize(Ct,[],options)
%     
%     result = strcmp(sol.info,str);
%     if result == 1
%         break
%     end       
end
Ct = [Ct, W>=0];
sol = optimize(Ct,[],options)

W = value(W);
for i = 1:numRule
    Y(:,:,i) = value(Y(:,:,i));
    K(:,:,i) = Y(:,:,i)/W;
end
%%
save ('gain_L.mat', 'K', 'Bbar', 'Abar', 'Dbar');

