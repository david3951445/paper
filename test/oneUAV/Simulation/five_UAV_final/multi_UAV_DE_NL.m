function [dy r1 r2 d1] = multi_UAV_DE(t,x,index)
  global Kx Ky Kz Kphi Ktheta Kpsi Jphi Jtheta Jpsi m g Ar
  global pv1 pv2 pv3 Local
  dy = zeros(length(x),1);
  r1 = zeros(12,1);
  r2 = zeros(12,1);
  d1 = zeros(4,1);
  d2 = zeros(4,1);
  v = 2*rand(length(x),1)-1;
  [K1 A1 B1 K2 A2 B2] = fuzzy_interpolation(x);
  w1 = index(1);  w2 = index(2);
  w3 = index(3);  w4 = index(4);

%% LEADER %%
   %% reference %%
  r1(1) = 1*sin(30*t*pi/180);
  r1(2) = (30*pi/180)*cos(30*t*pi/180);
  r1(3) = 1*cos(30*t*pi/180);
  r1(4) = -(30*pi/180)*sin(30*t*pi/180);
  d1(1) = (x(1)-r1(1));  %% x-xd
  d1(2) = x(2)-r1(2);    %% x_dot-x_dotd
  d1(3) = (x(3)-r1(3));  %% y-yd
  d1(4) = x(4)-r1(4);    %% y_dot-y_dotd
  ux = w1*d1(1)+w2*d1(2);
  uy = w3*d1(3)+w4*d1(4);
  r1(7) = 45*atan(ux*sin(r1(11))-uy*cos(r1(11)));                 %% roll
  r1(9) = -45*atan((ux*cos(r1(11))+uy*sin(r1(11)))/cos(r1(11)));  %% pitch
  r1(11) = 0;  %% pitch
  
  %% feedback input U = K*x %%
  U1 = K1*(x(1:12)-r1);
   %% dynamic %%
  dy(1) = x(2)+v(1);
  dy(2) = -(Kx/m)*x(2)+((cos(x(7)*pi/180)*sin(x(9)*pi/180)*cos(x(11)*pi/180)+sin(x(7)*pi/180)*sin(x(11)*pi/180))/m)*U1(1);%+v(2);
  dy(3) = x(4)+v(3);
  dy(4) = -(Ky/m)*x(4)+((cos(x(7)*pi/180)*sin(x(9)*pi/180)*sin(x(11)*pi/180)+sin(x(7)*pi/180)*cos(x(11)*pi/180))/m)*U1(1);%+v(4);
  dy(5) = x(6)+v(5);
  dy(6) = -(Kz/m)*x(6)-g+((cos(x(7)*pi/180)*cos(x(9)*pi/180))/m)*U1(1)+v(6);
  dy(7) = x(8)+v(7);
  dy(8) = -(Kphi/Jphi)*x(8)+(1/Jphi)*U1(2);%+v(8);
  dy(9) = x(10)+v(9);
  dy(10) = -(Ktheta/Jtheta)*x(10)+(1/Jtheta)*U1(3);%+v(10);
  dy(11) = x(12)+v(11);
  dy(12) = -(Kpsi/Jpsi)*x(12)+(1/Jpsi)*U1(4);%+v(12);

  %% FOLLOWER %%
  r2(1) = x(1)+0.5;
  r2(2) = x(2);
  r2(3) = x(3)+0.5;
  r2(4) = x(4);
  d2(1) = (x(13)-r2(1));  %% x-xd
  d2(2) = x(14)-r2(2);    %% x_dot-x_dotd
  d2(3) = (x(15)-r2(3));  %% y-yd
  d2(4) = x(16)-r2(4);    %% y_dot-y_dotd
  ux2 = w1*d2(1)+w2*d2(2);
  uy2 = w3*d2(3)+w4*d2(4);
  r2(7) = 45*atan(ux2*sin(r2(11))-uy2*cos(r2(11)));                 %% roll
  r2(9) = -45*atan((ux2*cos(r2(11))+uy2*sin(r2(11)))/cos(r2(11)));  %% pitch
  r2(11) = 0;  %% pitch
  %% feedback input U = K*x %%
  U2 = K2*(x(13:24)-r2);
   %% dynamic %%
  dy(13) = x(14)+v(13);
  dy(14) = -(Kx/m)*x(14)+((cos(x(19)*pi/180)*sin(x(21)*pi/180)*cos(x(23)*pi/180)+sin(x(19)*pi/180)*sin(x(23)*pi/180))/m)*U2(1);%+v(14);
  dy(15) = x(16)+v(15);
  dy(16) = -(Ky/m)*x(16)+((cos(x(19)*pi/180)*sin(x(21)*pi/180)*sin(x(23)*pi/180)+sin(x(19)*pi/180)*cos(x(23)*pi/180))/m)*U2(1);%+v(16);
  dy(17) = x(6)+v(17);
  dy(18) = -(Kz/m)*x(18)-g+((cos(x(19)*pi/180)*cos(x(21)*pi/180))/m)*U2(1)+v(18);
  dy(19) = x(20)+v(19);
  dy(20) = -(Kphi/Jphi)*x(20)+(1/Jphi)*U2(2);%+v(20);
  dy(21) = x(22)+v(21);
  dy(22) = -(Ktheta/Jtheta)*x(22)+(1/Jtheta)*U2(3);%+v(22);
  dy(23) = x(24)+v(23);
  dy(24) = -(Kpsi/Jpsi)*x(24)+(1/Jpsi)*U2(4);%+v(24);

end