function [x,y,len] = getEllipse(r1,r2,r3,r4,C)
beta = linspace(pi,-pi,2000);
len = length(beta);
for i = 1:len-1
    if beta(i)<pi && beta(i)>0
        x(i) = r1*cos(beta(i));
        y(i) = r2*sin(beta(i));
    else
        x(i) = r3*cos(beta(i));
        y(i) = r4*sin(beta(i));
    end
end
x = x + C(1,1);
y = y + C(1,2);
end