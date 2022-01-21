clc;clear;close all
%% load matrices
LP2 = load('lpv_data_part2_v2.mat');
LP3 = load('lpv_data_part3_v2.mat');

options = sdpsettings('solver','mosek');
options = sdpsettings(options,'verbose',0);
%% 
LP2.P = size(LP2.domain,1);
A2 = cell(size(LP2.S,1:LP2.P));
LP3.P = size(LP3.domain,1);
A3 = cell(size(LP3.S,1:LP3.P));

for i = 1:size(LP2.S,1)
    for j = 1:size(LP2.S,2)
        for k = 1:size(LP2.S,3)
            A2{i,j,k}(:,:) = LP2.S(i,j,k,:,:);
        end
    end
end

for i = 1:size(LP3.S,1)
    for j = 1:size(LP3.S,2)
        A3{i,j}(:,:) = LP3.S(i,j,:,:);
    end
end

%% parameter
S1 = [zeros(6,9),eye(6)];
S2 = [zeros(6,3),eye(6),zeros(6)];

beta = 10;

Ar = -beta*eye(15);
Br = beta*eye(15);
BrS = [zeros(7,15);-S2*Br];
Dbar = [...
    Br           zeros(15,13) ;
    BrS          eye(13)      ;
];
numRule = prod(size(LP2.S,1:LP2.P))*prod(size(LP3.S,1:LP3.P));
%% construct Abar
Abar = zeros(28,28,numRule);
c = 1;
for i1 = 1:size(LP2.S,1)
    for i2 = 1:size(LP2.S,2)
        for i3 = 1:size(LP2.S,3)
            for i4 = 1:size(LP3.S,1)
                for i5 = 1:size(LP3.S,2)
                    Abar(:,:,c) = [...
                        Ar, zeros(15,13);
                        zeros(4,3),A3{i4,i5},zeros(4,6),zeros(4,7),A3{i4,i5};
                        A2{i1,i2,i3};
                        S1-S2*Ar,zeros(6,13)
                        ];
                    c = c+1;
                end
            end
        end
    end
end

%% weighting matices 
Q1 = [...
    1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1
];
Q2 = [...
    1 0 0;
    0 1 0;
    0 0 1 
];
Q3 = [...
    1 0 0 0 0 0 ;
    0 1 0 0 0 0 ;
    0 0 1 0 0 0 ;
    0 0 0 1 0 0 ;
    0 0 0 0 1 0 ;
    0 0 0 0 0 1 
];
epsilon = 0.1;
Q = blkdiag(Q1,Q2,Q3);
QS = sqrtm(Q);
Qbar = 1e-8 * blkdiag(zeros(15),QS);
Rbar =  1e-8 * (eye(6) + epsilon^2*eye(6));
invRbar = inv(Rbar);
go = [zeros(22,6);eye(6)];
lo = 10;
dF = 1e-8;
D2 = Dbar*Dbar';
%% Lmi
K = zeros(6,28,numRule);
str = 'Infeasible problem (MOSEK)';
for i = 1:numRule
    %%% 目前缺少 Abar{-i,k} %%%
    Ct = 0;
    W = sdpvar(28,28);
    Y = sdpvar(6,28,'full');
    P11 = Abar(:,:,i)*W + W*Abar(:,:,i)' + go*Y + Y'*go' + 2*eye(28) + (1/lo)^2*D2;
    P21 = Y;
    P22 = -invRbar;
    P31 = Qbar*W;
    P32 = zeros(28,6);
    P33 = -eye(28);
    P41 = dF*W;
    P42 = zeros(28,6);
    P43 = zeros(28);
    P44 = -eye(28);


    LMI = [ P11 P21' P31' P41';...
            P21 P22  P32' P42';...
            P31 P32  P33  P43';...
            P41 P42  P43  P44];

    Ct = [Ct, LMI<=0];
    Ct = [Ct, W>=0];
    
    disp(i)
    
    sol = optimize(Ct,[],options)
    
    result = strcmp(sol.info,str);
    if result == 1
        break
    end
    
    W = value(W);
    Y = value(Y);
    K(:,:,i) = Y/W;
    
end

%%
save ('gain_F', 'K', 'go', 'Abar', 'Dbar');


for i = 1:numRule
    K(:,1:15,i) = zeros(6,15);
%     K(1,20,i) = 10*K(1,20,i);
%     K(1,21,i) = 10*K(1,21,i);
end

for i = 1:numRule
    E(:,i) = eig(Abar(:,:,i)+go*K(:,:,i));
end

