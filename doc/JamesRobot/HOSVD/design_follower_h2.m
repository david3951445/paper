clc;clear;close all
%% load matrices
LP2 = load('lpv_data_part2_v4.mat');
LP3 = load('lpv_data_part3_v4.mat');

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
S1 = [zeros(2,5),eye(2),zeros(2,4)];
S2 = [zeros(2,3),eye(2),zeros(2,6)];

beta = 10;

Ar = -beta*eye(11);
Br = beta*eye(11);
BrS = [zeros(7,11);-S2*Br;zeros(4,11)];
Dbar = [...
    Br           zeros(11,13) ;
    BrS          eye(13)      ;
];
numRule = prod(size(LP2.S,1:LP2.P))*prod(size(LP3.S,1:LP3.P));
%% construct Abar
Abar = zeros(24,24,numRule);
c = 1;
for i1 = 1:size(LP2.S,1)
    for i2 = 1:size(LP2.S,2)
        for i3 = 1:size(LP2.S,3)
            for i4 = 1:size(LP3.S,1)
                for i5 = 1:size(LP3.S,2)
                    Abar(:,:,c) = [...
                        Ar, zeros(11,13);
                        zeros(4,20),eye(4);
                        A2{i1,i2,i3};
                        S1-S2*Ar,zeros(2,13);
                        zeros(4,3),A3{i4,i5},zeros(4,3),eye(4),zeros(4,7),A3{i4,i5},zeros(4,5)
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
    10 0 0;
    0 10 0;
    0 0 10 
];
Q3 = [...
    1 0;
    0 1
];
Q4 = [...
    1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1
];
epsilon = 0.1;
Q = blkdiag(Q1,Q2,Q3,Q4);
QS = sqrtm(Q);
Qbar = 1e-5 * blkdiag(zeros(11),QS);
Rbar =  1e-4 * (eye(6) + epsilon^2*eye(6));
invRbar = inv(Rbar);
go = [zeros(18,6);eye(6)];
% lo = 10; % 1 success
dF = 1e-7;
D2 = Dbar*Dbar';
%% Lmi
K = zeros(6,24,numRule);
str = 'Infeasible problem (MOSEK)';
for i = 1:numRule
    %%% 目前缺少 Abar{-i,k} %%%
    Ct = 0;
    W = sdpvar(24,24);
    Y = sdpvar(6,24,'full');
    P11 = Abar(:,:,i)*W + W*Abar(:,:,i)' + go*Y + Y'*go' + 2*eye(24);
    P21 = Y;
    P22 = -invRbar;
    P31 = Qbar*W;
    P32 = zeros(24,6);
    P33 = -eye(24);
    P41 = dF*W;
    P42 = zeros(24,6);
    P43 = zeros(24);
    P44 = -eye(24);


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

save ('gain_F_h2', 'K', 'go', 'Abar', 'Dbar');
%%
% load 'gain_F'

% for i = 1:numRule
%     K(:,1:11,i) = zeros(6,11);
%     
%     % 讓 v 和 w 不受 q 影響
%     K(1:2,12:15,i) = zeros(2,4);
%     K(1:2,21:24,i) = zeros(2,4);
%     
    % 讓 q 不受 v 和 w 影響
%     K(3:6,16:20,i) = zeros(4,5);
    
    

%     K(:,16,i) = 10*K(:,16,i);
%     K(:,17,i) = 10*K(:,17,i);
%     K(2,16,i) = K(1,16,i);
%     K(:,17,i) = 6*K(:,17,i);
%     K(2,17,i) = K(1,17,i);
%     K(:,20,i) = K(:,20,i);
% end

% 
% for i = 1:numRule
%     E(:,i) = eig(Abar(:,:,i)+go*K(:,:,i));
% end

