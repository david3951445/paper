clc;clear;close all
%% load matrices
LP1 = load('lpv_data_part1_v2.mat');

options = sdpsettings('solver','mosek');
options = sdpsettings(options,'verbose',0);
%% part1 
LP1.P = size(LP1.domain,1);
A1 = cell(size(LP1.S,1:LP1.P));
for i = 1:size(LP1.S,1)
    for j = 1:size(LP1.S,2)
        for k = 1:size(LP1.S,3)
            for l = 1:size(LP1.S,4)
                A1{i,j,k,l}(:,:) = LP1.S(i,j,k,l,:,:);
            end
        end
    end
end


%% parameter
S3 = [eye(4) zeros(4,11)];
S5 = [0,1;1,0];
S4 = [zeros(2,8) S5 zeros(2,5)];
% S4 = [zeros(2,13) S5];

d = 0.2;
beta = 10;
Ar = -beta*eye(15);
Br = beta*eye(15);
BrS = [-S3*Br; -S4*Br];
Dbar = [...
    Br           zeros(15,6) ;
    BrS          eye(6)      ;
];
numRule = prod(size(LP1.S,1:LP1.P));
%% construct Abar
Abar = zeros(21,21,numRule);
c = 1;
for i1 = 1:size(LP1.S,1)
    for i2 = 1:size(LP1.S,2)
        for i3 = 1:size(LP1.S,3)
            for i4 = 1:size(LP1.S,4)
                Abar(:,:,c) = [...
                    Ar                                  zeros(15,4) zeros(15,2)                 ;
                    A1{i1,i2,i3,i4}(:,1:4)'*S4 - S3*Ar  zeros(4,4)  A1{i1,i2,i3,i4}(:,1:4)'     ;
                    A1{i1,i2,i3,i4}(:,5:19) - S4*Ar     zeros(2,4)  zeros(2)                    ;
                    ];
                c = c+1;
            end
        end
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
    1.5 0;
    0 1.5;
];
epsilon = 0.1;
Qbar = 1e-3 * blkdiag(zeros(15),Q1,Q2);
% QbarS = sqrtm(Qbar);
Rbar =  1e-1 * (eye(2) + epsilon^2*eye(2));
invRbar = inv(Rbar);
gbar = [...
    zeros(19,2) ;
    eye(2)      ;
];
lo = 10;
dF = 0.002;
D2 = Dbar*Dbar';
%% Lmi
K = zeros(2,21,numRule);
W = sdpvar(21,21);
Y = sdpvar(2,21,numRule,'full');
str = 'Infeasible problem (MOSEK)';
Ct = 0;
for i = 1:numRule
    %%% 目前缺少 Abar{-i,k} %%%
%     Ct = 0;
%     W = sdpvar(18,18);
%     Y = sdpvar(2,18,'full');
    
    P11 = Abar(:,:,i)*W + W*Abar(:,:,i)' + gbar*Y(:,:,i) + Y(:,:,i)'*gbar' + 2*eye(21) + (1/lo)^2*D2;
    P21 = Y(:,:,i);
    P22 = -invRbar;
    P31 = Qbar*W;
    P32 = zeros(21,2);
    P33 = -eye(21);
    P41 = dF*W;
    P42 = zeros(21,2);
    P43 = zeros(21);
    P44 = -eye(21);


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
% DDDD = Abar(:,:,6);
% DDD = Abar(:,:,6)+gbar*K(:,:,6);
% clc;
% C = eye(21);

% s = 5;

% [A,B,C,T,k] = ctrbf(Abar(:,:,s),gbar,C);

% rank(Abar(:,:,s));
% rank(ctrb(Abar(:,:,s),gbar));
% [V,J] = jordan(Abar(:,:,s));
% [V,D] = eig(Abar(:,:,s)+gbar*K(:,:,s));

%%
save ('gain_L.mat', 'K', 'gbar', 'Abar', 'Dbar');