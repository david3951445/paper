function [x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12]=refer(x,r)
global Ar Br
y=Ar*x+Br*r;
x1=y(1); x2=y(2); x3=y(3); x4=y(4);
x5=y(5); x6=y(6); x7=y(7); x8=y(8);
x9=y(9); x10=y(10); x11=y(11); x12=y(12);
end