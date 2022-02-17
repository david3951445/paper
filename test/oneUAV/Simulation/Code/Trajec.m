clc ;clear all;close all;
load control.mat
% Parameter Design
h=0.001;
T=60;
timeseries=h:h:T+h;
timeseries1=h:h:T;

global Ar Br Kp Kd Ki
global  Kx Ky Kz Kphi Ktheta Kpsi Jx Jy Jz m g

Kp(2:4,1:6)=10^(-13)*Kp(2:4,1:6);
Kp(1,3)=1*Kp(1,3);
Kp(2:4,1:3)=0.00001*Kp(2:4,1:3);
Kp(1,1:6)=0.3*Kp(1,1:6);
Kp(1,1:2)=0.5*Kp(1,1:2);
Kp(2:4,4:6)=5*Kp(2:4,4:6);
clear sum
% Kp(3,4)=0;
% Kp(3,6)=0;
% Kp(2,5)=0;
% Kp(2,6)=0;
% Kp(4,4)=0;
% Kp(4,5)=0;
Kp(1,4:6)=0.01*Kp(1,4:6);


Ki=1*10^(4)*Ki;
Ki(2:4,1:3)=zeros(3,3);
Ki(2:4,4:6)=0.0001*Ki(2:4,4:6);
Ki(1,4:6)=zeros(1,3);

Ki(1,3)=-30*Ki(1,3);
% Ki(1,1:2)=zeros(1,2);
% Ki(1,4:5)=zeros(1,2);
% Ki(1,6)=1*Ki(1,6);



Kd(2:4,1:6)=10^(-3)*Kd(2:4,1:6);
Kd(2:4,4:6)=1000*Kd(2:4,4:6);
Kd(2:4,1:3)=0.001*Kd(2:4,1:3);
Kd(2:4,4:6)=4*Kd(2:4,4:6);
Kd(4,6)=0.5*Kd(4,6);
Kd(1,3)=0.05*Kd(1,3);
Kd(1:3,1)=0.5*Kd(1:3,1);
Kd(1,4:6)=Kd(1:3,1);
Kd(1,3)=1*Kd(1,3);
Kd(4,6)=1*Kd(4,6);
Kd(1,4)=0;
%% Construction of Wiener Process
Wie1=WieSource(h,10*T/h);
Wie2=WieSource(h,10*T/h);
Wie3=WieSource(h,10*T/h);
Poi1=Poisson(T,h,0.1);
Poi2=Poisson(T,h,0.1);
Poi3=Poisson(T,h,0.1);
%% Reference Model

for i=1:1:T/h;
freq=0.05;
amp=5;
xp(i)=amp*sin(2*pi*freq*h*i);% 2*pi*f*t
xp1(i)=amp*sin(2*pi*freq*h*i)+2;% 2*pi*f*t
xp2(i)=amp*sin(2*pi*freq*h*i)+4;% 2*pi*f*t
xv(i)=amp*2*pi*freq*cos(2*pi*freq*h*i);
%y
yp(i)=amp*cos(2*pi*freq*h*i);
yp1(i)=amp*cos(2*pi*freq*h*i)+2;
yp2(i)=amp*cos(2*pi*freq*h*i)+4;
yv(i)=-amp*2*pi*freq*sin(2*pi*freq*h*i);
%z
zp(i)=15+2*i*h;
zv(i)=2;
zp1(i)=14+2*i*h;
zp2(i)=16+2*i*h;


end



%% Generate reference
xr(:,1)=[0 yp(1) zp(1) 0 0 0 0 0 0 0 0 0];
xr1(:,1)=[0 yp1(1) zp1(1) 0 0 0 0 0 0 0 0 0];
xr2(:,1)=[0 yp2(1) zp2(1) 0 0 0 0 0 0 0 0 0];
xuav(:,1)=[xp(1) yp(1) zp(1) 0 0 0 0 0 0 0 0 0];
xuav1(:,1)=[xp1(1) yp1(1) zp1(1) 0 0 0 0 0 0 0 0 0];
xuav2(:,1)=[xp2(1) yp2(1) zp2(1) 0 0 0 0 0 0 0 0 0];
for i=1:1:T/h
% Setting the time-invariant varying reference of first UAV
r(1,i)=xp(i);r(2,i)=yp(i);r(3,i)=zp(i);
r(7,i)=xv(i);r(8,i)=yv(i);r(9,i)=zv(i);
r(6,i)=0;r(12,i)=0; 
% Setting the time-invariant varying reference of 2nd UAV
r1(1,i)=xp1(i);r1(2,i)=yp1(i);r1(3,i)=zp1(i);
r1(7,i)=xv(i);r1(8,i)=yv(i);r1(9,i)=zv(i);
r1(6,i)=0;r1(12,i)=0; 
% Setting the time-invariant varying reference of 3rd UAV
r2(1,i)=xp2(i);r2(2,i)=yp2(i);r2(3,i)=zp2(i);
r2(7,i)=xv(i);r2(8,i)=yv(i);r2(9,i)=zv(i);
r2(6,i)=0;r2(12,i)=0; 
% Setting the time-invariant varying reference os 1st UAV
con=20;
dx=con*(0.5*(xuav(1,i)-xr(1,i))+0.5*(xuav(7,i)-xr(7,i)));
dy=con*(0.5*(xuav(2,i)-xr(2,i))+0.5*(xuav(8,i)-xr(8,i)));
dz=con*(0.5*(xuav(3,i)-xr(3,i))+0.5*(xuav(9,i)-xr(9,i)));
r(4,i)=0.4*atan(m*(-dy)/(dx+dy+dz));
r(5,i)=0.4*atan(dx/(dz+1*g));
if i==1
r(10,i)=r(4,i)/h;r(11,:)=r(5,i)/h;  
else
  r(10,i)=(r(4,i)-r(4,i-1))/h;
  r(11,i)=(r(5,i)-r(5,i-1))/h; 
end
% Setting the time-invariant varying reference os 2nd UAV
con=20;
dx1=con*(0.5*(xuav1(1,i)-xr1(1,i))+0.5*(xuav1(7,i)-xr1(7,i)));
dy1=con*(0.5*(xuav1(2,i)-xr1(2,i))+0.5*(xuav1(8,i)-xr1(8,i)));
dz1=con*(0.5*(xuav1(3,i)-xr1(3,i))+0.5*(xuav1(9,i)-xr1(9,i)));
r1(4,i)=0.4*atan(m*(-dy1)/(dx1+dy1+dz1));
r1(5,i)=0.4*atan(dx1/(dz1+1*g));
if i==1
r1(10,i)=r1(4,i)/h;r1(11,:)=r1(5,i)/h;  
else
  r1(10,i)=(r1(4,i)-r1(4,i-1))/h;
  r1(11,i)=(r1(5,i)-r1(5,i-1))/h; 
end    
% Setting the time-invariant varying reference os 3rd UAV
con=20;
dx2=con*(0.5*(xuav2(1,i)-xr2(1,i))+0.5*(xuav2(7,i)-xr2(7,i)));
dy2=con*(0.5*(xuav2(2,i)-xr2(2,i))+0.5*(xuav2(8,i)-xr2(8,i)));
dz2=con*(0.5*(xuav2(3,i)-xr2(3,i))+0.5*(xuav2(9,i)-xr2(9,i)));
r2(4,i)=0.4*atan(m*(-dy2)/(dx2+dy2+dz2));
r2(5,i)=0.4*atan(dx2/(dz2+1*g));
if i==1
r2(10,i)=r2(4,i)/h;r2(11,:)=r2(5,i)/h;  
else
  r2(10,i)=(r2(4,i)-r2(4,i-1))/h;
  r2(11,i)=(r2(5,i)-r2(5,i-1))/h; 
end        
%Disturbance
v(:,i)=[0.01*xuav1(7,i)*xuav1(7,i)+0.01*xuav2(7,i)*xuav2(7,i) 0.01*xuav1(8,i)*xuav1(8,i)+0.01*xuav2(8,i)*xuav2(8,i) ...
        0.01*xuav1(9,i)*xuav1(9,i)+0.01*xuav2(9,i)*xuav2(9,i) zeros(1,3)];
v1(:,i)=[0.01*xuav(7,i)*xuav(7,i)+0.01*xuav2(7,i)*xuav2(7,i) 0.01*xuav(8,i)*xuav(8,i)+0.01*xuav2(8,i)*xuav2(8,i) ...
        0.01*xuav(9,i)*xuav(9,i)+0.01*xuav2(9,i)*xuav2(9,i) zeros(1,3)];
v2(:,i)=[0.01*xuav(7,i)*xuav(7,i)+0.01*xuav1(7,i)*xuav1(7,i) 0.01*xuav(8,i)*xuav(8,i)+0.01*xuav1(8,i)*xuav1(8,i) ...
        0.01*xuav(9,i)*xuav(9,i)+0.01*xuav1(9,i)*xuav1(9,i) zeros(1,3)];
% Integral Part
if i==1
xi(:,i)=((xuav(1:6,i)-xr(1:6,i))*h);  
xi1(:,i)=((xuav1(1:6,i)-xr1(1:6,i))*h);  
xi2(:,i)=((xuav2(1:6,i)-xr2(1:6,i))*h);  
else
xi(:,i)=(xi(:,i-1)+(xuav(1:6,i)-xr(1:6,i))*h);
xi1(:,i)=(xi1(:,i-1)+(xuav1(1:6,i)-xr1(1:6,i))*h);
xi2(:,i)=(xi2(:,i-1)+(xuav2(1:6,i)-xr2(1:6,i))*h);
end
% Step 1 Reference
 [k11 k12 k13 k14 k15 k16 k17 k18 k19 k110 k111 k112]=refer(xr(:,i),r(:,i));
kk11= h*[k11 k12 k13 k14 k15 k16 k17 k18 k19 k110 k111 k112]';
y=xr(:,i)+kk11/2;

 [rk11 rk12 rk13 rk14 rk15 rk16 rk17 rk18 rk19 rk110 rk111 rk112]=refer(xr1(:,i),r1(:,i));
rkk11= h*[rk11 rk12 rk13 rk14 rk15 rk16 rk17 rk18 rk19 rk110 rk111 rk112]';
ry=xr1(:,i)+rkk11/2;

 [rrk11 rrk12 rrk13 rrk14 rrk15 rrk16 rrk17 rrk18 rrk19 rrk110 rrk111 rrk112]=refer(xr2(:,i),r2(:,i));
rrkk11= h*[rrk11 rrk12 rrk13 rrk14 rrk15 rrk16 rrk17 rrk18 rrk19 rrk110 rrk111 rrk112]';
rry=xr2(:,i)+rrkk11/2;

% Step 1 UAV
 [uk11 uk12 uk13 uk14 uk15 uk16 uk17 uk18 uk19 uk110 uk111 uk112]=uavsys(xuav(:,i),xr(:,i),v(:,i),xi(:,i));
ukk11= h*[uk11 uk12 uk13 uk14 uk15 uk16 uk17 uk18 uk19 uk110 uk111 uk112]';
uy=xuav(:,i)+ukk11/2;

 [ruk11 ruk12 ruk13 ruk14 ruk15 ruk16 ruk17 ruk18 ruk19 ruk110 ruk111 ruk112]=uavsys(xuav1(:,i),xr1(:,i),v1(:,i),xi1(:,i));
rukk11= h*[ruk11 ruk12 ruk13 ruk14 ruk15 ruk16 ruk17 ruk18 ruk19 ruk110 ruk111 ruk112]';
ruy=xuav1(:,i)+rukk11/2;

 [rruk11 rruk12 rruk13 rruk14 rruk15 rruk16 rruk17 rruk18 rruk19 rruk110 rruk111 rruk112]=uavsys(xuav2(:,i),xr2(:,i),v2(:,i),xi2(:,i));
rrukk11= h*[rruk11 rruk12 rruk13 rruk14 rruk15 rruk16 rruk17 rruk18 rruk19 rruk110 rruk111 rruk112]';
rruy=xuav2(:,i)+rrukk11/2;


% Step 2 Reference
[k21 k22 k23 k24 k25 k26 k27 k28 k29 k210 k211 k212]=refer(y,r(:,i));
kk21= h*[k21 k22 k23 k24 k25 k26 k27 k28 k29 k210 k211 k212]';
y1=xr(:,i)+kk21/2; 

[rk21 rk22 rk23 rk24 rk25 rk26 rk27 rk28 rk29 rk210 rk211 rk212]=refer(ry,r1(:,i));
rkk21= h*[rk21 rk22 rk23 rk24 rk25 rk26 rk27 rk28 rk29 rk210 rk211 rk212]';
ry1=xr1(:,i)+rkk21/2; 

[rrk21 rrk22 rrk23 rrk24 rrk25 rrk26 rrk27 rrk28 rrk29 rrk210 rrk211 rrk212]=refer(rry,r2(:,i));
rrkk21= h*[rrk21 rrk22 rrk23 rrk24 rrk25 rrk26 rrk27 rrk28 rrk29 rrk210 rrk211 rrk212]';
rry1=xr2(:,i)+rrkk21/2; 


% Step2 UAV
 [uk21 uk22 uk23 uk24 uk25 uk26 uk27 uk28 uk29 uk210 uk211 uk212]=uavsys(uy,xr(:,i),v(:,i),xi(:,i));
ukk21= h*[uk11 uk12 uk13 uk14 uk15 uk16 uk17 uk18 uk19 uk110 uk111 uk112]';
uy1=xuav(:,i)+ukk21/2;

 [ruk21 ruk22 ruk23 ruk24 ruk25 ruk26 ruk27 ruk28 ruk29 ruk210 ruk211 ruk212]=uavsys(ruy,xr1(:,i),v1(:,i),xi1(:,i));
rukk21= h*[ruk21 ruk22 ruk23 ruk24 ruk25 ruk26 ruk27 ruk28 ruk29 ruk210 ruk211 ruk212]';
ruy1=xuav1(:,i)+rukk21/2;

 [rruk21 rruk22 rruk23 rruk24 rruk25 rruk26 rruk27 rruk28 rruk29 rruk210 rruk211 rruk212]=uavsys(rruy,xr2(:,i),v2(:,i),xi2(:,i));
rrukk21= h*[rruk21 rruk22 rruk23 rruk24 rruk25 rruk26 rruk27 rruk28 rruk29 rruk210 rruk211 rruk212]';
rruy1=xuav2(:,i)+rrukk21/2;

%Step 3 Reference
[k31 k32 k33 k34 k35 k36 k37 k38 k39 k310 k311 k312]=refer(y1,r(:,i));
kk31= h*[k31 k32 k33 k34 k35 k36 k37 k38 k39 k310 k311 k312]';
y2=xr(:,i)+kk31; 

[rk31 rk32 rk33 rk34 rk35 rk36 rk37 rk38 rk39 rk310 rk311 rk312]=refer(ry1,r1(:,i));
rkk31= h*[rk31 rk32 rk33 rk34 rk35 rk36 rk37 rk38 rk39 rk310 rk311 rk312]';
ry2=xr1(:,i)+rkk31; 

[rrk31 rrk32 rrk33 rrk34 rrk35 rrk36 rrk37 rrk38 rrk39 rrk310 rrk311 rrk312]=refer(rry1,r2(:,i));
rrkk31= h*[rrk31 rrk32 rrk33 rrk34 rrk35 rrk36 rrk37 rrk38 rrk39 rrk310 rrk311 rrk312]';
rry2=xr1(:,i)+rrkk31; 

% Step3 UAV
 [uk31 uk32 uk33 uk34 uk35 uk36 uk37 uk38 uk39 uk310 uk311 uk312]=uavsys(uy1,xr(:,i),v(:,i),xi(:,i));
ukk31= h*[uk31 uk32 uk33 uk34 uk35 uk36 uk37 uk38 uk39 uk310 uk311 uk312]';
uy2=xuav(:,i)+ukk31;

 [ruk31 ruk32 ruk33 ruk34 ruk35 ruk36 ruk37 ruk38 ruk39 ruk310 ruk311 ruk312]=uavsys(ruy1,xr1(:,i),v1(:,i),xi1(:,i));
rukk31= h*[ruk31 ruk32 ruk33 ruk34 ruk35 ruk36 ruk37 ruk38 ruk39 ruk310 ruk311 ruk312]';
ruy2=xuav1(:,i)+rukk31;

 [rruk31 rruk32 rruk33 rruk34 rruk35 rruk36 rruk37 rruk38 rruk39 rruk310 rruk311 rruk312]=uavsys(rruy1,xr2(:,i),v2(:,i),xi2(:,i));
rrukk31= h*[rruk31 rruk32 rruk33 rruk34 rruk35 rruk36 rruk37 rruk38 rruk39 rruk310 rruk311 rruk312]';
rruy2=xuav2(:,i)+rrukk31;


%Step 4 Reference
[k41 k42 k43 k44 k45 k46 k47 k48 k49 k410 k411 k412]=refer(y2,r(:,i));
kk41=h*[k41 k42 k43 k44 k45 k46 k47 k48 k49 k410 k411 k412]';

[rk41 rk42 rk43 rk44 rk45 rk46 rk47 rk48 rk49 rk410 rk411 rk412]=refer(ry2,r1(:,i));
rkk41=h*[rk41 rk42 rk43 rk44 rk45 rk46 rk47 rk48 rk49 rk410 rk411 rk412]';

[rrk41 rrk42 rrk43 rrk44 rrk45 rrk46 rrk47 rrk48 rrk49 rrk410 rrk411 rrk412]=refer(rry2,r2(:,i));
rrkk41=h*[rrk41 rrk42 rrk43 rrk44 rrk45 rrk46 rrk47 rrk48 rrk49 rrk410 rrk411 rrk412]';
%Step 4 UAV
 [uk41 uk42 uk43 uk44 uk45 uk46 uk47 uk48 uk49 uk410 uk411 uk412]=uavsys(uy2,xr(:,i),v(:,i),xi(:,i));
ukk41= h*[uk41 uk42 uk43 uk44 uk45 uk46 uk47 uk48 uk49 uk410 uk411 uk412]';

 [ruk41 ruk42 ruk43 ruk44 ruk45 ruk46 ruk47 ruk48 ruk49 ruk410 ruk411 ruk412]=uavsys(ruy2,xr1(:,i),v1(:,i),xi1(:,i));
rukk41= h*[ruk41 ruk42 ruk43 ruk44 ruk45 ruk46 ruk47 ruk48 ruk49 ruk410 ruk411 ruk412]';

[rruk41 rruk42 rruk43 rruk44 rruk45 rruk46 rruk47 rruk48 rruk49 rruk410 rruk411 rruk412]=uavsys(rruy2,xr2(:,i),v2(:,i),xi2(:,i));
rrukk41= h*[rruk41 rruk42 rruk43 rruk44 rruk45 rruk46 rruk47 rruk48 rruk49 rruk410 rruk411 rruk412]';

Winc1 = sum(Wie1(8*(i-1)+1:8*i));
Winc2 = sum(Wie2(8*(i-1)+1:8*i));
Winc3 = sum(Wie3(8*(i-1)+1:8*i));
% Next Step
xr(:,i+1)=xr(:,i)+(kk11+2*kk21+2*kk31+kk41)/6;   
xr1(:,i+1)=xr1(:,i)+(rkk11+2*rkk21+2*rkk31+rkk41)/6;
xr2(:,i+1)=xr2(:,i)+(rrkk11+2*rrkk21+2*rrkk31+rrkk41)/6;
xuav(:,i+1)=xuav(:,i)+(ukk11+2*ukk21+2*ukk31+ukk41)/6+Winc1*blkdiag(zeros(6,6),1*10^(-3)*eye(3),zeros(3,3))*xuav(:,i);  
xuav1(:,i+1)=xuav1(:,i)+(rukk11+2*rukk21+2*rukk31+rukk41)/6+Winc2*blkdiag(zeros(6,6),1*10^(-3)*eye(3),zeros(3,3))*xuav1(:,i); ;  
xuav2(:,i+1)=xuav2(:,i)+(rrukk11+2*rrukk21+2*rrukk31+rrukk41)/6+Winc3*blkdiag(zeros(6,6),1*10^(-3)*eye(3),zeros(3,3))*xuav2(:,i);   
if xuav(1,i)>1000
    break
end
u(1:4,i)=Kp*(xuav(1:6,i)-xr(1:6,i))+Kd*(xuav(7:12,i)-xr(7:12,i))+Ki*xi(1:6,i);
if Poi1(i)>0
    xuav(:,i+1)=xuav(:,i)+(ukk11+2*ukk21+2*ukk31+ukk41)/6+Winc1*blkdiag(zeros(6,6),1*10^(-5)*eye(3),zeros(3,3))*xuav(:,i)...
        +blkdiag(zeros(6,6),zeros(3,3),5*10^(-1)*eye(3))*xuav(:,i);  
end
if Poi2(i)>0
    xuav1(:,i+1)=xuav1(:,i)+(ukk11+2*ukk21+2*ukk31+ukk41)/6+Winc1*blkdiag(zeros(6,6),1*10^(-5)*eye(3),zeros(3,3))*xuav1(:,i)...
        +blkdiag(zeros(6,6),zeros(3,3),5*10^(-1)*eye(3))*xuav1(:,i);  
end
if Poi3(i)>0
    xuav2(:,i+1)=xuav2(:,i)+(ukk11+2*ukk21+2*ukk31+ukk41)/6+Winc1*blkdiag(zeros(6,6),1*10^(-5)*eye(3),zeros(3,3))*xuav2(:,i)...
        +blkdiag(zeros(6,6),zeros(3,3),5*10^(-1)*eye(3))*xuav2(:,i);  
end
i
end
%% Plot of Three UAVs
close all
figure (1)
subplot(2,1,1)
plot(timeseries,xuav(1,:))
hold on
plot(timeseries,xuav1(1,:))
hold on
plot(timeseries,xuav2(1,:))
axis([0 60 -10 10])
  leg=xlabel('$t$')
  set(leg,'Interpreter','latex');
  leg=ylabel('$m$')
   set(leg,'Interpreter','latex');
leg63=title('$\:(a) X-Axis$')
  set(leg63,'Interpreter','latex');
%  set(leg63,'FontSize',9); % Set Font Size
subplot(2,1,2)
plot(timeseries,xuav(7,:))
hold on
plot(timeseries,xuav1(7,:))
hold on
plot(timeseries,xuav2(7,:))
leg=legend('$UAV 1$','$UAV 2$','$UAV 3$')
set(leg,'Interpreter','latex');
axis([0 60 -3 3])
  leg=xlabel('$t$')
  set(leg,'Interpreter','latex');
  leg=ylabel('$m/s$')
  set(leg,'Interpreter','latex');
leg63=title('$(b)\:Velocity\: on\: X-Axis$')
  set(leg63,'Interpreter','latex');


figure (2)
subplot(2,1,1)
plot(timeseries,xuav(2,:))
hold on
plot(timeseries,xuav1(2,:))
hold on
plot(timeseries,xuav2(2,:))
axis([0 60 -10 10])
  leg=xlabel('$t$')
  set(leg,'Interpreter','latex');
  leg=ylabel('$m$')
   set(leg,'Interpreter','latex');
leg63=title('$\:(a) Y-Axis$')
  set(leg63,'Interpreter','latex');
%  set(leg63,'FontSize',9); % Set Font Size
subplot(2,1,2)
plot(timeseries,xuav(8,:))
hold on
plot(timeseries,xuav1(8,:))
hold on
plot(timeseries,xuav2(8,:))
leg=legend('$UAV 1$','$UAV 2$','$UAV 3$')
set(leg,'Interpreter','latex');
axis([0 60 -3 3])
  leg=xlabel('$t$')
  set(leg,'Interpreter','latex');
  leg=ylabel('$m/s$')
  set(leg,'Interpreter','latex');
leg63=title('$(b)\:Velocity\: on\: Y-Axis$')
  set(leg63,'Interpreter','latex');

figure (3)
subplot(2,1,1)
plot(timeseries,xuav(3,:))
hold on
plot(timeseries,xuav1(3,:))
hold on
plot(timeseries,xuav2(3,:))
axis([0 60 -20 150])
  leg=xlabel('$t$')
  set(leg,'Interpreter','latex');
  leg=ylabel('$m$')
   set(leg,'Interpreter','latex');
leg63=title('$\:(a) Z-Axis$')
  set(leg63,'Interpreter','latex');
%  set(leg63,'FontSize',9); % Set Font Size
subplot(2,1,2)
plot(timeseries,xuav(9,:))
hold on
plot(timeseries,xuav1(9,:))
hold on
plot(timeseries,xuav2(9,:))
leg=legend('$UAV 1$','$UAV 2$','$UAV 3$')
set(leg,'Interpreter','latex');
axis([0 60 -3 3])
  leg=xlabel('$t$')
  set(leg,'Interpreter','latex');
  leg=ylabel('$m/s$')
  set(leg,'Interpreter','latex');
leg63=title('$(b)\:Velocity\: on\: Z-Axis$')
  set(leg63,'Interpreter','latex');

figure (4)
subplot(2,1,1)
plot(timeseries,xuav(4,:))
hold on
plot(timeseries,xuav1(4,:))
hold on
plot(timeseries,xuav2(4,:))
axis([0 60 -0.25 0.25])
  leg=xlabel('$t$')
  set(leg,'Interpreter','latex');
  leg=ylabel('$rad$')
   set(leg,'Interpreter','latex');
leg63=title('$\:(a) Angle\: \phi$')
  set(leg63,'Interpreter','latex');
%  set(leg63,'FontSize',9); % Set Font Size
subplot(2,1,2)
plot(timeseries,xuav(10,:))
hold on
plot(timeseries,xuav1(10,:))
hold on
plot(timeseries,xuav2(10,:))
leg=legend('$UAV 1$','$UAV 2$','$UAV 3$')
set(leg,'Interpreter','latex');
axis([0 60 -0.25 0.25])
  leg=xlabel('$t$')
  set(leg,'Interpreter','latex');
  leg=ylabel('$rad/s$')
  set(leg,'Interpreter','latex');
leg63=title('$(b)\:Angular\:Velocity \: V_{\psi}$')
  set(leg63,'Interpreter','latex');


figure (5)
subplot(2,1,1)
plot(timeseries,xuav(5,:))
hold on
plot(timeseries,xuav1(5,:))
hold on
plot(timeseries,xuav2(5,:))
axis([0 60 -0.25 0.25])
  leg=xlabel('$t$')
  set(leg,'Interpreter','latex');
  leg=ylabel('$rad$')
   set(leg,'Interpreter','latex');
leg63=title('$\:(a) Angle\: \theta$')
  set(leg63,'Interpreter','latex');
%  set(leg63,'FontSize',9); % Set Font Size
subplot(2,1,2)
plot(timeseries,xuav(11,:))
hold on
plot(timeseries,xuav1(11,:))
hold on
plot(timeseries,xuav2(11,:))
leg=legend('$UAV 1$','$UAV 2$','$UAV 3$')
set(leg,'Interpreter','latex');
axis([0 60 -0.5 0.5])
  leg=xlabel('$t$')
  set(leg,'Interpreter','latex');
  leg=ylabel('$rad/s$')
  set(leg,'Interpreter','latex');
leg63=title('$(b)\:Angular\:Velocity \: V_{\theta}$')
  set(leg63,'Interpreter','latex');

figure (6)
subplot(2,1,1)
plot(timeseries,xuav(6,:))
hold on
plot(timeseries,xuav1(6,:))
hold on
plot(timeseries,xuav2(6,:))
axis([0 60 -0.25 0.25])
  leg=xlabel('$t$')
  set(leg,'Interpreter','latex');
  leg=ylabel('$rad$')
   set(leg,'Interpreter','latex');
leg63=title('$\:(a) Angle\: \psi$')
  set(leg63,'Interpreter','latex');
%  set(leg63,'FontSize',9); % Set Font Size
subplot(2,1,2)
plot(timeseries,xuav(12,:))
hold on
plot(timeseries,xuav1(12,:))
hold on
plot(timeseries,xuav2(12,:))
leg=legend('$UAV 1$','$UAV 2$','$UAV 3$')
set(leg,'Interpreter','latex');
axis([0 60 -0.25 0.25])
  leg=xlabel('$t$')
  set(leg,'Interpreter','latex');
  leg=ylabel('$rad/s$')
  set(leg,'Interpreter','latex');
leg63=title('$(b)\:Angular\:Velocity \: V_{\psi}$')
  set(leg63,'Interpreter','latex');
  %%
figure (7)
plot3(xuav(1,:),xuav(2,:),xuav(3,:))
hold on
plot3(xuav1(1,:),xuav1(2,:),xuav1(3,:))
hold on
plot3(xuav2(1,:),xuav2(2,:),xuav2(3,:))
grid on
leg=legend('$UAV 1$','$UAV 2$','$UAV 3$')
set(leg,'Interpreter','latex');
leg63=title('$3D\:Graph\:of\:Three\:UAVs$')
  set(leg63,'Interpreter','latex');
    leg=xlabel('$m$')
  set(leg,'Interpreter','latex');
      leg=ylabel('$m$')
  set(leg,'Interpreter','latex');
      leg=zlabel('$m$')
  set(leg,'Interpreter','latex');
  %%

figure (8) % Control of 1st UAV
subplot(2,2,1)
plot(timeseries1,u(1,:))
grid on
leg=legend('$F_{1}$')
set(leg,'Interpreter','latex');
leg63=title('$F_{1}\:of\:1st\:UAV$')
  set(leg63,'Interpreter','latex');
    leg=xlabel('$t$')
  set(leg,'Interpreter','latex');
      leg=ylabel('$N$')
  set(leg,'Interpreter','latex');

subplot(2,2,2)
plot(timeseries1,u(2,:))
grid on
leg=legend('$\tau_{1,\phi}$')
set(leg,'Interpreter','latex');
leg63=title('$\tau_{1,\phi}\:of\:1st\:UAV$')
  set(leg63,'Interpreter','latex');
    leg=xlabel('$t$')
  set(leg,'Interpreter','latex');
      leg=ylabel('$NM$')
  set(leg,'Interpreter','latex');
subplot(2,2,3)
plot(timeseries1,u(3,:))
grid on
leg=legend('$\tau_{1,\theta}$')
set(leg,'Interpreter','latex');
leg63=title('$\tau_{1,\theta}\:of\:1st\:UAV$')
  set(leg63,'Interpreter','latex');
     leg=xlabel('$t$')
  set(leg,'Interpreter','latex');
      leg=ylabel('$NM$')
  set(leg,'Interpreter','latex');
  axis([0 60 -10 10])
subplot(2,2,4)
plot(timeseries1,u(4,:))
leg=legend('$\tau_{1,\psi}$')
set(leg,'Interpreter','latex');
leg63=title('$\tau_{1,\psi}\:of\:1st\:UAV$')
  set(leg63,'Interpreter','latex');
     leg=xlabel('$t$')
  set(leg,'Interpreter','latex');
      leg=ylabel('$NM$')
  set(leg,'Interpreter','latex');
grid on
%% Calculation of pho
pho(1)=0;pho(2)=0;pho(3)=0;
for i=1:1:T/h
    if i==1;
nu1(i)=dot((xuav(:,i)-r(:,i)),(xuav(:,i)-r(:,i)))*h;
nu2(i)=dot((xuav1(:,i)-r1(:,i)),(xuav1(:,i)-r1(:,i)))*h;
nu3(i)=dot((xuav2(:,i)-r2(:,i)),(xuav2(:,i)-r2(:,i)))*h;
denu1(i)=dot((r(:,i)),(r(:,i)))*h;
denu2(i)=dot((r1(:,i)),(r1(:,i)))*h;
denu3(i)=dot((r2(:,i)),(r2(:,i)))*h;
    else
      nu1(i)=nu1(i-1)+(dot((xuav(:,i)-r(:,i)),(xuav(:,i)-r(:,i))))*h;
nu2(i)=nu2(i-1)+(dot((xuav1(:,i)-r1(:,i)),(xuav1(:,i)-r1(:,i))))*h;
nu3(i)=nu3(i-1)+(dot((xuav2(:,i)-r2(:,i)),(xuav2(:,i)-r2(:,i))))*h;
denu1(i)=denu1(i-1)+(dot((r(:,i)),(r(:,i))))*h;
denu2(i)=denu2(i-1)+(dot((r1(:,i)),(r1(:,i))))*h;
denu3(i)=denu3(i-1)+(dot((r2(:,i)),(r2(:,i))))*h;  
    end
end
pho1=nu1(T/h)/denu1(T/h)
pho2=nu2(T/h)/denu2(T/h)
pho3=nu3(T/h)/denu3(T/h)