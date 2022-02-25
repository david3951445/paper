%%% from frame{0} to frame{4} %%% 
function A = leg_FK(th,L3,L4,L5)
d = zeros(1,4);
a = zeros(1,4);
afa = zeros(1,4);

th(2) = th(2)-pi/2;
d(1) = -L3;
a(3) = L4;
a(4) = L5;
afa(1) = pi/2;
afa(2) = -pi/2;

A = eye(4);
for i = 1:4
    A = A*leg_T(th(i),d(i),a(i),afa(i));
end

end