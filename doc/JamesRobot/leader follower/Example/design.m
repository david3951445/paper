%% Construction of System Matrix
clc 
clear all
close all
%% Yalmip Root Setting
% addpath(genpath('C:\Program Files\MATLAB\R2017a\toolbox\Yalmip\YALMIP-R20181012')) %¾Ç®Õ
% addpath(genpath('C:\Program Files\Mosek\8\toolbox\r2014a')) % mosekdiag
options = sdpsettings('solver','mosek');
options = sdpsettings(options,'verbose',0);
load matrices.mat
%% Setting of Weighting
Qinf=blkdiag(0*eye(4,4),0*eye(4,4),0.001*eye(8,8));
Q2=blkdiag(0*eye(4,4),0*eye(4,4),0.001*eye(8,8));
alpha=20;
beta=4;
%% Lmi of first system
W1=sdpvar(16,16);
W2=sdpvar(16,16);
Y=sdpvar(16,2,'full');
Ct=0;
% H2 constraint
P11=W1*Abar+Abar'*W1+Gbar'*W1*Gbar+Gbar'*W2*Gbar+Jbar'*W1*Jbar+Jbar'*W2*Jbar+W1*Jbar+Jbar'*W1;
P21=W2*Jbar;
P22=W2*Abar-Y*Cbar+Abar'*W2-Cbar'*Y'+Q2;
LMI1=[P11 P21' ;
      P21 P22];
Ct = [Ct, LMI1<=0];
% Initial constraint 
LMI2=[W1 zeros(16);
      zeros(16) W2];
Ct= [ Ct, LMI2<=alpha*eye(32)];
% H inf constraint
Q11=W1*Abar+Abar'*W1+Gbar'*W1*Gbar+Gbar'*W2*Gbar+Jbar'*W1*Jbar+Jbar'*W2*Jbar+W1*Jbar+Jbar'*W1;
Q21=W2*Jbar;
Q22=W2*Abar-Y*Cbar+Abar'*W2-Cbar'*Y'+Qinf;

Q13=W1*Bbar;
Q14=zeros(16,14);
Q23=W2*Bbar;
Q24=Y*Dbar;

Q33=-beta*eye(14);
Q34=zeros(14,14);
Q44=-beta*eye(14);
LMI3=[ Q11 Q21' Q13 Q14;...
      Q21 Q22 Q23 Q24;...
      Q13' Q23' Q33' Q34;
      Q14' Q24' Q34' Q44];
  Ct = [Ct, LMI3<=0];


Ct = [Ct, W1>=0];
Ct = [Ct, W2>=0];

% Solver 
sol = optimize(Ct,[],options)
W1=value(W1);
W2=value(W2);
Y=value(Y);
L=inv(W2)*Y;
eig(Abar-L*Cbar)


