clear all
close all
clc
%% Yalmip Root Setting
addpath(genpath('C:\Program Files\MATLAB\R2017a\toolbox\Yalmip\YALMIP-R20181012')) %¾Ç®Õ
addpath(genpath('C:\Program Files\Mosek\8\toolbox\r2014a')) % mosekdiag
options = sdpsettings('solver','mosek');
options = sdpsettings(options,'verbose',0);
%% Construction of System Matrix
load model.mat 
% Parameter Setting
pho=1;
Q=0.01*eye(30);
% Ar=blkdiag(zeros(6,6),-1*eye(6));
Ar=-4*eye(12);
Br=-Ar;
T1=[eye(6) zeros(6,6)];
T2=[zeros(6,6) eye(6)];

T=[T1 zeros(6,18);
  T2  zeros(6,18);
  zeros(6,24) eye(6)];
T3=[eye(6) zeros(6,6)];
Ddis=[zeros(6,6);eye(6)];
Daug=0*[Ddis -Br;zeros(12,6) Br; zeros(6,18)];
for i=1:length(z2)
    for j=1:length(phi1)
        for k=1:length(phi2)
             for l=1:length(theta1)
                  for n=1:length(theta2)
                      
       Aaug(i,j,k,l,n).Aaug=[Asys(i,j,k,l,n).Asys Asys(i,j,k,l,n).Asys-Ar zeros(12,6);
                             zeros(12,12) Ar zeros(12,6);
                             T3 zeros(6,12) zeros(6,6)]; 
       Baug(i,j,k,l,n).Baug=[Bsys(i,j,k,l,n).Bsys;zeros(18,4)];

          
                  end
             end
        end
    end
end
%% LMI condition
W11=sdpvar(6,6);
W12=sdpvar(6,6);
W2=sdpvar(12,12);
W3=sdpvar(6,6);
Y1=sdpvar(4,6,'full');
Y2=sdpvar(4,6,'full');
Y3=sdpvar(4,6,'full');
Ct=0;
low=10^(-10);
Ct=[Ct,W11>=low];Ct=[Ct,W12>=low];
Ct=[Ct,W2>=low];Ct=[Ct,W3>=low];
% Ct=[Ct,W11>=0];Ct=[Ct,W12>=0];
% Ct=[Ct,W2>=0];Ct=[Ct,W3>=0];
sum=0;
for i=1:length(z2)
    for j=1:length(phi1)
        for k=1:length(phi2)
             for l=1:length(theta1)
                  for n=1:length(theta2)
         W=blkdiag(W11,W12,W2,W3);
         Y=[Y1 Y2 zeros(4,12) Y3];        
         lmi1=Aaug(i,j,k,l,n).Aaug*W+Baug(i,j,k,l,n).Baug*Y;
         lmi11=lmi1+lmi1'+Daug*Daug'/pho;
         lmi12=W*sqrt(Q);
         lmi22=-eye(30);
         lmi=[lmi11 lmi12 ;...
             lmi12' lmi22];     
        sum=sum+1;
         Ct=[Ct,lmi<=0];
        
                  end
             end
        end
    end
end
sol = optimize(Ct,[],options)
W11=value(W11);W2=value(W2);
W12=value(W12);W3=value(W3);
Y1=value(Y1);Y2=value(Y2);Y3=value(Y3);
Kp=Y1*inv(W11);Kd=Y2*inv(W12);Ki=Y3*inv(W3);
save control.mat