function [r, F] = r_F(obj, x, t)
%Generate reference and feedforward force
% x     | vector | system state
% t     | scalar | current time
% r     | vector | tracking reference
% F     | scalar | feedforward force
amp_z   = 1; % 0.8

amp     = 1; % 1
freg    = 0.5;

xd      = amp*sin(freg*t);
dxd     = freg*amp*cos(freg*t);
d2xd    = -freg^2*amp*sin(freg*t);
yd      = amp*cos(freg*t);
dyd     = -freg*amp*sin(freg*t);
d2yd    = -freg^2*amp*cos(freg*t);
zd      = amp_z*t;
dzd     = amp_z;
d2zd    = 0;
psid    = 0;

r = [
    xd
    dxd
    yd
    dyd
    zd
    dzd
    0
    0
    0
    0
    psid
    psid
];

%% method 1, 1.pdf, r = r(x, t)
% method 1-1
% ux = obj.c(1)*(x(1) - y(1)) + obj.c(2)*(x(2) - y(2));
% uy = obj.c(3)*(x(3) - y(3)) + obj.c(4)*(x(4) - y(4));
% uz = obj.c(5)*(x(5) - y(5)) + obj.c(6)*(x(6) - y(6));
        
% a = 1/(uz + obj.G);
% phi_d = x(11);
% r(7) = asin(obj.m/F*(ux*sin(x(11)) - uy*cos(x(11))));
% r(7) = atan(a*(ux*sin(phi_d) - uy*cos(phi_d)));
% r(9) = atan(a*(ux*cos(phi_d) + uy*sin(phi_d))/cos(x(7)));

% method 1-3
% y(7) = 0.4*atan(obj.m*(-uy)/(ux + uy + uz));
% y(9) = 0.4*atan(ux/(uz + obj.G));

% method 1-4 (李博's method)         
% y(7) = atan(ux*sin(y(11))-1*uy*cos(y(11))); % roll
% % y(8) = (y(7)-f1(7))/h;
% y(9) = atan((ux*cos(y(11))+uy*sin(y(11)))/cos(y(11))); % pitch
% % y(10) = (y(9)-f1(9))/h;

%% method 2, inverse dynamic, r = r(t)
% How to find x(7), x(9)
%       - UAV is an underactuated system. Lets say UAV has 6 DOF (degree of freedom) x, y, z, phi, theta, psi.
%         If 4 DOF is assigned, other 2 DOF will be restricted. 
%       - Sub xd, yd, zd, psid into dx/dt = f(x) + g(x)*u. phid, theatad, F can
%         be found by inverse dynamic.

% method 2-1 (good)
% r(7) = atan(-freg*cos(freg*t)/obj.G);
% r(9) = atan(freg*sin(freg*t)/(obj.G*cos(r(7))));
% F = 0;

% method 2-2 (good)
c1 = obj.m*d2xd + obj.Kx*dxd;
c2 = obj.m*d2yd + obj.Ky*dyd;
c3 = obj.m*(d2zd + obj.G) + obj.Kz*dzd;
r(7) = atan2(c1, c3);
r(9) = atan2(-c2*cos(r(7)), c3);
F = sqrt(c1^2 + c2^2 + c3^2);
end