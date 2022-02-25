function [p,v,a] = com0_linearize(B,dC,h,T)
p = dC*h+B;
v = dC/T;
a = 0;
end