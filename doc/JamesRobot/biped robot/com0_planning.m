function [p,v,a] = com0_planning(A,B,C,D,E,F,T,hh)
%% a to a' (linear)
tacc = 7;
T1 = T-tacc;
t = 0:hh:T1;
t1 = length(t);
dB = B-A;
h = t/T;
for i = 1:t1
    [p(i),v(i),a(i)] = com0_linearize(A,dB,h(i),T);
end
A1 = p(t1);
%% a' to b' (quadratic)
T2 = T+tacc;
t = T1:hh:T2;
t2 = length(t);
dB1 = B-A1;
dC = C-B;
h = (t-T1)/(2*tacc);
t_pass = t1-1;
for i = 1:t2
    [p(i+t_pass),v(i+t_pass),a(i+t_pass)] = com0_quadratic(B,dB1,dC,tacc,h(i),T);
end
%% b' to b'' (linear)
T3 = 2*T-tacc;
t = T2:hh:T3;
t3 = length(t);
h = (t-T)/T;
t_pass = t_pass+t2-1;
for i = 1:t3
    [p(i+t_pass),v(i+t_pass),a(i+t_pass)] = com0_linearize(B,dC,h(i),T);
end
B1 = p(t3+t_pass);
%% b'' to c' (quadratic)
T4 = 2*T+tacc;
t = T3:hh:T4;
t4 = length(t);
dC1 = C-B1;
dD = D-C;
h = (t-T3)/(2*tacc);
t_pass = t_pass+t3-1;
for i = 1:t4
    [p(i+t_pass),v(i+t_pass),a(i+t_pass)] = com0_quadratic(C,dC1,dD,tacc,h(i),T);
end
%% c' to c'' (linear)
T5 = 3*T-tacc;
t = T4:hh:T5;
t5 = length(t);
h = (t-2*T)/T;
t_pass = t_pass+t4-1;
for i = 1:t5
    [p(i+t_pass),v(i+t_pass),a(i+t_pass)] = com0_linearize(C,dD,h(i),T);
end
C1 = p(t5+t_pass);
%% c'' to d' (quadratic)
T6 = 3*T+tacc;
t = T5:hh:T6;
t6 = length(t);
dD1 = D-C1;
dE = E-D;
h = (t-T5)/(2*tacc);
t_pass = t_pass+t5-1;
for i = 1:t6
    [p(i+t_pass),v(i+t_pass),a(i+t_pass)] = com0_quadratic(D,dD1,dE,tacc,h(i),T);
end

%% d' to d'' (linear)
T7 = 4*T-tacc;
t = T6:hh:T7;
t7 = length(t);
h = (t-3*T)/T;
t_pass = t_pass+t6-1;
for i = 1:t7
    [p(i+t_pass),v(i+t_pass),a(i+t_pass)] = com0_linearize(D,dE,h(i),T);
end
D1 = p(t7+t_pass);
%% d'' to e' (quadratic)
T8 = 4*T+tacc;
t = T7:hh:T8;
t8 = length(t);
dE1 = E-D1;
dF = F-E;
h = (t-T7)/(2*tacc);
t_pass = t_pass+t7-1;
for i = 1:t8
    [p(i+t_pass),v(i+t_pass),a(i+t_pass)] = com0_quadratic(E,dE1,dF,tacc,h(i),T);
end
%% e' to f (linear)
T9 = 5*T;
t = T8:hh:T9;
t9 = length(t);
h = (t-4*T)/T;
t_pass = t_pass+t8-1;
for i = 1:t9
    [p(i+t_pass),v(i+t_pass),a(i+t_pass)] = com0_linearize(E,dF,h(i),T);
end
D1 = p(t7+t_pass);
end