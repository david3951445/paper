clc;clear;close all
%% load matrices
LP3 = load('lpv_data_part3_v4.mat');

options = sdpsettings('solver','mosek');
options = sdpsettings(options,'verbose',0);
%% 
LP3.P = size(LP3.domain,1);
A3 = cell(size(LP3.S,1:LP3.P));

for i = 1:size(LP3.S,1)
    for j = 1:size(LP3.S,2)
        A1{i,j}(:,:) = LP3.S(i,j,:,:);
    end
end

%% parameter
beta = 10;

Ar = -beta*eye(5);
Br = beta*eye(5);
Dbar = [...
    Br                 zeros(5,8) ;
    zeros(8,5)         eye(8)      ;
];
numRule = prod(size(LP3.S,1:LP3.P));
%% construct Abar
Abar = zeros(13,13,numRule);
c = 1;
for i1 = 1:size(LP3.S,1)
    for i2 = 1:size(LP3.S,2)
        Abar(:,:,c) = [...
            Ar, zeros(5,8);
            zeros(4,5),zeros(4),eye(4);
            A1{i1,i2},eye(4),zeros(4,8)
            ];
        c = c+1;
    end
end

%% weighting matices 
Q1 = [...
    10 0 0 0;
    0 10 0 0;
    0 0 10 0;
    0 0 0 10
];
Q2 = [...
    1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1
];
epsilon = 0.1;
Q = blkdiag(Q1,Q2);
QS = sqrtm(Q);
Qbar = 1e-1 * blkdiag(zeros(5),QS);
Rbar =  1e-7 * (eye(4) + epsilon^2*eye(4));
invRbar = inv(Rbar);
go = [zeros(9,4);eye(4)];
lo = 10;
dF = 1e-7;
D2 = Dbar*Dbar';
%% Lmi
K = zeros(4,13,numRule);
str = 'Infeasible problem (MOSEK)';
for i = 1:numRule
    %%% 目前缺少 Abar{-i,k} %%%
    Ct = 0;
    W = sdpvar(13,13);
    Y = sdpvar(4,13,'full');
    P11 = Abar(:,:,i)*W + W*Abar(:,:,i)' + go*Y + Y'*go' + 2*eye(13) + (1/lo)^2*D2;
    P21 = Y;
    P22 = -invRbar;
    P31 = Qbar*W;
    P32 = zeros(13,4);
    P33 = -eye(13);
    P41 = dF*W;
    P42 = zeros(13,4);
    P43 = zeros(13);
    P44 = -eye(13);


    LMI = [ P11 P21' P31' P41';...
            P21 P22  P32' P42';...
            P31 P32  P33  P43';...
            P41 P42  P43  P44];

    Ct = [Ct, LMI<=0];
    Ct = [Ct, W>=0];
    
    disp(i);
    
    sol = optimize(Ct,[],options)
    
    result = strcmp(sol.info,str);
    if result == 1
        break
    end
    
    W = value(W);
    Y = value(Y);
    K(:,:,i) = Y/W;
    
end

save ('gain_F', 'K', 'go', 'Abar', 'Dbar');
%%
load 'gain_F'

for i = 1:numRule
    K(:,1:5,i) = zeros(4,5);
end


for i = 1:numRule
    E(:,i) = eig(Abar(:,:,i)+go*K(:,:,i));
end

