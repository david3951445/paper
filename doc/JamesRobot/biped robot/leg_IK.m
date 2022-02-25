%%% from frame{0} to frame{4} %%% 
% 不能用atan，會產生NaN
function th = leg_IK(A,L3,L4,L5)
% unit: rad
n = A(:,1);
o = A(:,2);
a = A(:,3);
p = A(:,4);
%%%  -pi/2 < th < pi/2 %%%
num1 = a(2);
den1 = a(1);
th1 = atan2(num1,den1);
% th1 = atan(num1/den1);

num2 = cos(th1)*p(1)+sin(th1)*p(2);
den2 = p(3)+L3;
th2 = atan2(num2,-den2);
% th2 = atan(-num2/den2);

f1 = cos(th1)*p(1)+sin(th1)*p(2);
f2 = p(3)+L3;
f3 = sin(th1)*p(1)-cos(th1)*p(2);
num4 = f1^2 + f2^2 + f3^2 - L4^2 - L5^2;
den4 = 2*L4*L5;
th4 = acos(num4/den4);

g1 = cos(th1)*sin(th2)*p(1) + sin(th1)*sin(th2)*p(2) - cos(th2)*p(3) - L3*cos(th2);
g2 = -sin(th1)*p(1) + cos(th1)*p(2);
num3 = g2*(L4+L5*cos(th4)) - g1*L5*sin(th4);
den3 = g1*(L4+L5*cos(th4)) + g2*L5*sin(th4);
th3 = atan2(num3,den3);
% th3 = atan(num3/den3);

th = [th1 th2 th3 th4];
end
