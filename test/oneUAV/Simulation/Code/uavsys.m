function [x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12]=uavsys(x,xr,v,xi)
global Ar Br Kp Kd Ki
global  Kx Ky Kz Kphi Ktheta Kpsi Jx Jy Jz m g
u=Kp*(x(1:6)-xr(1:6))+Kd*(x(7:12)-xr(7:12))+Ki*xi;
%posion
x1=x(7);x2=x(8);
x3=x(9);x4=x(10);
x5=x(11);x6=x(12);
% Velocity
x7=-Kx*x(7)/m+(cos(x(4))*sin(x(5))*cos(x(6))+sin(x(4))*sin(x(6)))*u(1)/m+v(1);
x8=-Ky*x(8)/m+(cos(x(4))*sin(x(5))*sin(x(6))-sin(x(4))*cos(x(6)))*u(1)/m+v(2);
x9=-Kz*x(9)/m-g+cos(x(4))*cos(x(5))*u(1)/m+v(3);
x10=(Jy-Jz)*x(5)*x(6)/Jx-Kphi*x(10)/Jz+u(2)/Jx+v(4);
x11=(Jz-Jx)*x(4)*x(6)/Jy-Ktheta*x(11)/Jy+u(3)/Jy+v(5);
x12=(Jx-Jy)*x(4)*x(5)/Jz-Kpsi*x(12)/Jz+u(4)/Jz+v(6);
end