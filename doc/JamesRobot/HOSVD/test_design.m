clc;clear;close all
% Result: failed (out of memory)
%% load matrices
LP1 = load('lpv_data_part1.mat');
LP2 = load('lpv_data_part2.mat');
LP3 = load('lpv_data_part3.mat');

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


%% part2
LP2.P = size(LP2.domain,1);
A2 = cell(size(LP2.S,1:LP2.P),1);
for i = 1:size(LP2.S,1)
    A2{i}(:,:) = LP2.S(i,:,:);
end

%% part3
LP3.P = size(LP3.domain,1);
A3 = cell(size(LP3.S,1:LP3.P));
for i = 1:size(LP3.S,1)
    for j = 1:size(LP3.S,2)
        for k = 1:size(LP3.S,3)
            for l = 1:size(LP3.S,4)
                for m = 1:size(LP3.S,5)
                    for n = 1:size(LP3.S,6)
                        A3{i,j,k,l,m,n}(:,:) = LP3.S(i,j,k,l,m,n,:,:);
                    end
                end
            end
        end
    end
end

%% parameter
S1 = [eye(8) zeros(8,4)];
S2 = [zeros(1,2) eye(1)];
S3 = [eye(4) zeros(4,8)];
S4 = [S1' [zeros(4,2) eye(4) [zeros(2) eye(2)]' eye(4)]']';
S5 = [[eye(2) zeros(2)]' [zeros(1,3) eye(1)]']';
S7 = [S5 zeros(3,2)];
S6 = [zeros(6,12) S7' zeros(6,12)]';
S9 = [zeros(4,2) -eye(4) [zeros(2) -eye(2)]']';
S8 = blkdiag(eye(8),S9)';

d = 20;
beta=10;
Ar=-beta*eye(12);
Br=beta*eye(12);
BrS = [-S3*Br; zeros(2,12)];
D = [...
    Br           zeros(12,6) zeros(12,27);
    BrS          eye(6)      zeros(6,27) ;
    zeros(27,12) -S6         eye(27)
];
numRule = prod(size(LP1.S,1:LP1.P))*prod(size(LP2.S,1:LP2.P))*prod(size(LP3.S,1:LP3.P));
%% construct Abar
Abar = zeros(45,45,numRule);
c = 1;
for i1 = 1:size(LP1.S,1)
    for i2 = 1:size(LP1.S,2)
        for i3 = 1:size(LP1.S,3)
            for i4 = 1:size(LP1.S,4)
                for i5 = 1:size(LP2.S,1)
                    for i6 = 1:size(LP3.S,1)
                        for i7 = 1:size(LP3.S,2)
                            for i8 = 1:size(LP3.S,3)
                                for i9 = 1:size(LP3.S,4)
                                    for i10 = 1:size(LP3.S,5)
                                        for i11 = 1:size(LP3.S,6)
                                            Abar(:,:,c) = [...
                                                Ar                                zeros(12,4) zeros(12,2)                 zeros(12)   zeros(12,3) zeros(12)                       ;
                                                S3*Ar                             zeros(4,4)  A1{i1,i2,i3,i4}(:,1:4)'     zeros(4,12) zeros(4,3)  zeros(4,12)                     ;
                                                A1{i1,i2,i3,i4}(:,5:16)           zeros(2,4)  zeros(2)                    zeros(2,12) zeros(2,3)  zeros(2,12)                     ;
                                                zeros(12)                         zeros(12,4) zeros(12,2)                 zeros(12)   zeros(12,3) A3{i6,i7,i8,i9,i10,i11}(1:12,:) ;
                                                zeros(3,12)                       zeros(3,4)  A1{i1,i2,i3,i4}(:,17:19)'   zeros(3,12) zeros(3)    A2{i5}(:,:)                     ;
                                                A3{i6,i7,i8,i9,i10,i11}(13:24,:)  zeros(12,4) zeros(12,2)                 zeros(12)   zeros(12,3) zeros(12)
                                                ];
                                            c = c+1; 
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
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
    0 0 0 0 0 0 0 0 0 0 0 0 ;
    0 0 0 0 0 0 0 0 0 0 0 0 ;
    0 0 0 0 0 0 0 0 0 0 0 0 ;
    0 0 0 0 0 0 0 0 0 0 0 0 ;
    0 0 0 0 0 0 0 0 0 0 0 0 ;
    0 0 0 0 0 0 0 0 0 0 0 0 ;
    0 0 0 0 0 0 0 0 0 0 0 0 ;
    0 0 0 0 0 0 0 0 0 0 0 0 ;
    0 0 0 0 0 0 0 0 1 0 0 0 ;
    0 0 0 0 0 0 0 0 0 1 0 0 ;
    0 0 0 0 0 0 0 0 0 0 1 0 ;
    0 0 0 0 0 0 0 0 0 0 0 1 
];
Q3 = [...
    1 0 0;
    0 1 0;
    0 0 1
];
epsilon = 0.01;
Qbar = 1e-3 * blkdiag(zeros(12),Q1,zeros(2),Q2,Q3,zeros(12));
% QbarS = sqrtm(Qbar);
Rbar =  1e-3 * (blkdiag(eye(14)) + epsilon^2*eye(14));
invRbar = inv(Rbar);
gbar = [...
    zeros(16,2) zeros(16,12);
    eye(2)      zeros(2,12) ;
    zeros(15,2) zeros(15,12);
    zeros(12,2) eye(12)    
];
lo = 100;
dF = 0.001;
D2 = D*D';
%% Lmi
echo = 1;

matsize = 500;
rem = mod(numRule,matsize);
matdim = (numRule-rem)/matsize;
K = cell(matdim,1);
W = cell(matdim,1);
Y = cell(matdim,1);
for i = 1:matdim
K{i} = zeros(14,45,matsize);
W{i} = sdpvar(45,45,matsize);
Y{i} = sdpvar(14,45,matsize,'full');
disp(i)
end
K{matdim+1} = zeros(14,45,rem);
W{matdim+1} = sdpvar(45,45,rem);
Y{matdim+1} = sdpvar(14,45,rem,'full');

save 'designmatrice.mat'
%%
load 'designmatrice.mat'
P22 = -invRbar;
P32 = zeros(45,14);
P33 = -eye(45);
P42 = zeros(45,14);
P43 = zeros(45);
P44 = -eye(45);

str = 'Infeasible problem (MOSEK)';
Ct = 0;
for i = 1:numRule
    for j = 1:matdim+1
        if i <= matsize*j
            P11 = Abar(:,:,i)*W{j}(:,:,i-matsize*(j-1)) + W{j}(:,:,i-matsize*(j-1))*Abar(:,:,i)' + gbar*Y{j}(:,:,i-matsize*(j-1))+ Y{j}(:,:,i-matsize*(j-1))'*gbar' + 2*eye(45) + (1/lo)^2*D2;
            P21 = Y{j}(:,:,i-matsize*(j-1));
            P31 = Qbar*W{j}(:,:,i-matsize*(j-1));
            P41 = dF*W{j}(:,:,i-matsize*(j-1));
            LMI = [ P11 P21' P31' P41';...
                P21 P22 P32' P42';...
                P31 P32 P33  P43';...
                P41 P42 P43  P44];

            Ct = [Ct, LMI<=0];
    
            Ct = [Ct, W{j}(:,:,i-matsize*(j-1))>=0];
            break;
        end
    end
    disp(i)
end
sol = optimize(Ct,[],options)