clc;clear;close all
%% load matrices
LP1 = load('lpv_data_part1_v8.mat');

options = sdpsettings('solver','mosek');
options = sdpsettings(options,'verbose',0);
%% part1 
LP1.P = size(LP1.domain,1);
A1 = cell(size(LP1.S,1:LP1.P));
for i = 1:size(LP1.S,1)
    for j = 1:size(LP1.S,2)
        for k = 1:size(LP1.S,3)
            for l = 1:size(LP1.S,4)
                for m = 1:size(LP1.S,5)
                    for n = 1:size(LP1.S,6)
                        A1{i,j,k,l,m,n}(:,:) = LP1.S(i,j,k,l,m,n,:,:);
                    end
                end
            end
        end
    end
end


%% parameter
S1 = [zeros(2,7),eye(2)];
S2 = [zeros(2,4),eye(2),zeros(2,3)];

d = 0.2;
beta = 10;

Ar = -beta*eye(9);
Br = beta*eye(9);
BrS = [zeros(4,9);-S2*Br];
Dbar = [...
    Br           zeros(9,6) ;
    BrS          eye(6)      ;
];
numRule = prod(size(LP1.S,1:LP1.P));
%% construct Abar
Abar = zeros(15,15,numRule);
c = 1;
for i1 = 1:size(LP1.S,1)
    for i2 = 1:size(LP1.S,2)
        for i3 = 1:size(LP1.S,3)
            for i4 = 1:size(LP1.S,4)
                for i5 = 1:size(LP1.S,5)
                    for i6 = 1:size(LP1.S,6)
                        Abar(:,:,c) = [...
                            Ar, zeros(9,6);
                            A1{i1,i2,i3,i4,i5,i6};
                            S1 - S2*Ar, zeros(2,6)
                            ];
                        c = c+1;
                    end
                end
            end
        end
    end
end

%% weighting matices 
Q1 = [...
    5 0 0 0 ;
    0 5 0 0 ;
    0 0 1 0 ;
    0 0 0 1
];
Q2 = [...
    1 0;
    0 1;
];
epsilon = 0.1;
Q = blkdiag(Q1,Q2);
QS = sqrtm(blkdiag(Q1,Q2));
Qbar = 1e-3 * blkdiag(zeros(9),QS);
Rbar =  1e-3 * (eye(2) + epsilon^2*eye(2));
invRbar = inv(Rbar);
go = [zeros(13,2);eye(2)];
% lo = 1000000000; % 1 success
dF = 0.001;
D2 = Dbar*Dbar';
%% Lmi
K = zeros(2,15,numRule);
str = 'Infeasible problem (MOSEK)';
for i = 1:numRule
    %%% 目前缺少 Abar{-i,k} %%%
    Ct = 0;
    W = sdpvar(15,15);
    Y = sdpvar(2,15,'full');
    P11 = Abar(:,:,i)*W + W*Abar(:,:,i)' + go*Y + Y'*go' + 2*eye(15);
    P21 = Y;
    P22 = -invRbar;
    P31 = Qbar*W;
    P32 = zeros(15,2);
    P33 = -eye(15);
    P41 = dF*W;
    P42 = zeros(15,2);
    P43 = zeros(15);
    P44 = -eye(15);


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
save ('gain_L_h2', 'K', 'go', 'Abar', 'Dbar');


for i = 1:numRule
    K(:,1:9,i) = zeros(2,9);
    K(1,10,i) = 3*K(1,10,i);
    K(2,10,i) = 0.1*K(1,10,i);
    K(1,11,i) = 3*K(1,11,i);
    K(2,11,i) = 0.1*K(1,11,i);
end

for i = 1:numRule
    E(:,i) = eig(Abar(:,:,i)+go*K(:,:,i));
end

