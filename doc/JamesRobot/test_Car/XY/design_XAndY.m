clc;clear;close all
%% load matrices
LP1 = load('lpv_data.mat');

options = sdpsettings('solver','mosek');
options = sdpsettings(options,'verbose',0);
%% part1 
LP1.P = size(LP1.domain,1);
A1 = cell(size(LP1.S,1:LP1.P),1);
for i = 1:size(LP1.S,1)
    A1{i}(:,:) = LP1.S(i,:,:);
end

%% parameter

S1 = [eye(2),zeros(2,5)];
S2 = [zeros(1,2) eye(1) zeros(1,4)];
S3 = [zeros(2,3) [1,0;0,1] zeros(2)];
S4 = [zeros(1,4) eye(1) zeros(1,2)];
S5 = [zeros(2,5) eye(2)];
S6 = [zeros(1,3) eye(1) zeros(1,3)];


d = 0.2;
beta = 10;
Ar = -beta*eye(7);
Br = beta*eye(7);
BrS = [-S1*Br;-S2*Br;-S3*Br];
Dbar = [...
    Br           zeros(7,5) ;
    BrS          eye(5)      ;
];
numRule = prod(size(LP1.S,1:LP1.P));
%% construct Abar
Abar = zeros(12,12,numRule);
c = 1;
for i = 1:size(LP1.S,1)
    Abar(:,:,c) = [...
        Ar                         zeros(7,3)  zeros(7,1)   zeros(7,1)     ;
        A1{i}'*S6 - S1*Ar          zeros(2,3)  A1{i}'       zeros(2,1)     ;
        S4 - S2*Ar                 zeros(1,3)  zeros(1,1)   eye(1)         ;
        S5 - S3*Ar                 zeros(2,3)  zeros(2,1)   zeros(2,1)
        ];
    c = c+1;
end

%% weighting matices
Q1 = [...
    4 0 0;
    0 4 0;
    0 0 4
];
Q2 = [...
    1 0;
    0 1
];
epsilon = 0.1;
% Qbar = 1e-3 * blkdiag(zeros(7),Q1,Q2);
QS = sqrtm(blkdiag(Q1,Q2));
Qbar = 1e-3 * blkdiag(zeros(7),QS);
Rbar =  1e-2 * (eye(2) + epsilon^2*eye(2));
invRbar = inv(Rbar);
go = [zeros(10,2);eye(2)];
lo = 10;
dF = 0.001;
D2 = Dbar*Dbar';
%% Lmi
K = zeros(2,12,numRule);
% W = sdpvar(12,12);
% Y = sdpvar(2,12,numRule,'full');
str = 'Infeasible problem (MOSEK)';
% Ct = 0;
for i = 1:numRule
    %%% 目前缺少 Abar{-i,k} %%%
    Ct = 0;
    W = sdpvar(12,12);
    Y = sdpvar(2,12,'full');
    
    P11 = Abar(:,:,i)*W + W*Abar(:,:,i)' + go*Y + Y'*go' + 2*eye(12) + (1/lo)^2*D2;
    P21 = Y;
    P22 = -invRbar;
    P31 = Qbar*W;
    P32 = zeros(12,2);
    P33 = -eye(12);
    P41 = dF*W;
    P42 = zeros(12,2);
    P43 = zeros(12);
    P44 = -eye(12);


    LMI = [ P11 P21' P31' P41';...
            P21 P22  P32' P42';...
            P31 P32  P33  P43';...
            P41 P42  P43  P44];

    Ct = [Ct, LMI<=0];
    Ct = [Ct, W>=0];
    sol = optimize(Ct,[],options)
    result = strcmp(sol.info,str);
    if result == 1
        break
    end   
    
    W = value(W);
    Y = value(Y);
    K(:,:,i) = Y/W;
    disp(i)     
end
% Ct = [Ct, W>=0];
% sol = optimize(Ct,[],options)
% 
% W = value(W);
% for i = 1:numRule
%     Y(:,:,i) = value(Y(:,:,i));
%     K(:,:,i) = Y(:,:,i)/W;
% end
%%
save ('gain_L.mat', 'K', 'go', 'Abar', 'Dbar', 'numRule');
for i = 1:numRule
    K(:,1:7,i) = zeros(2,7);
end
for i = 1:numRule
    E(:,i) = eig(Abar(:,:,i)+go*K(:,:,i));
end