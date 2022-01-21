% test defuzzy method
%% trapmf
clc;clear;

mem(1,:) = [-inf -inf 0 1];
mem(2,:) = [0 1 1 2];
mem(3,:) = [1 2 inf inf];


v = -1:0.1:3;
H = trapmf(v,mem(2,:));
plot(v,H)
% 
% for j = 1:3
% H(j) = trapmf(v,mem(j,:));
% end

%% test
clc;clear;

mem1 = [-1 0 1];
mem2 = [0 1 2];
U1 = [0 1 0]';
U2 = U1;

x = -0.5;


H1 = trimf(x,mem1);
H2 = trimf(x,mem2);


h1 = mf(x,mem1,U1);
h2 = mf(x,mem2,U2);