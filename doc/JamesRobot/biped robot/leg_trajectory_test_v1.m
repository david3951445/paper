clc;clear;
close all
load com0_reference_model.mat
r1 = 0.1;
r2 = 0.25;
r4 = 0;
[x,z,len] = getEllipse(r1,r2,r1,r4,[r1 0]);
tl = 1:len-1;
dz = diff(z);
dz = [dz(1) dz];
ddz = diff(dz);
ddz = [ddz(1) ddz];
len2 = 1000;

zr(:,1) = zeros(3,1);
Ar = -bata*eye(3);
rz = [z;dz;ddz]; % reference input
for i = 1:len2-1
    k1 = com0_RungeKutta(Ar,zr(:,i),bata*rz(:,2*i-1));
    k2 = com0_RungeKutta(Ar,zr(:,i)+hh*k1/2,bata*rz(:,2*i));
    k3 = com0_RungeKutta(Ar,zr(:,i)+hh*k2/2,bata*rz(:,2*i));
    k4 = com0_RungeKutta(Ar,zr(:,i)+hh*k3,bata*rz(:,2*i+1));
    zr(:,i+1) = zr(:,i) + 1/6*hh*(k1+2*k2+2*k3+k4);
end

tl2 = 1:len2;









figure 
plot(x,z);
axis equal tight 
% figure 
% plot(tl2,x);
figure 
plot(tl2,zr(1,:));
figure 
plot(tl2,zr(2,:));
figure 
plot(tl2,zr(3,:));
fx(1) = x(1);
fz(1) = z(1);


flagx = 0;
flagz = 0;
% para = ((1/2*(a*(1-z^2/b))^(-1/2))*(-a)/b*(2*z))^-1;
for i = 1:length(tt)
    if mod(flagx,2)==0
        fx(i+1) = fx(i)+V(i)*hh;
    else
        fx(i+1) = fx(i)-V(i)*hh;
    end
    
%     para(i) = ((1/2*(r1^2*(1-(fz(i)^2)/(r2^2)))^(-1/2))*(-r1^2)/(r2^2)*(2*fz(i)))^-1;
    if mod(flagz,2)==0 && mod(flagx,2)==0
%         fz(i+1) = fz(i)+V(i)*para(i)*hh;
        fz(i+1) = ((1 - fx(i+1)/r1^2)*r2^2)^(1/2);
    elseif mod(flagz,2)==1 && mod(flagx,2)==0
%         fz(i+1) = fz(i)-V(i)*para(i)*hh;
        fz(i+1) = ((1 - fx(i+1)/r1^2)*r2^2)^(1/2);  
    else
        fz(i+1) = 0;
    end
    
    
    if fx(i+1)>r1 || fx(i+1)<-r1
        flagx = flagx+1;
    end
    
    if fz(i+1)>=r2 || (fx(i+1)<-r1 && fz(i+1) == 0)
        flagz = flagz+1;
    end
end

figure 
subplot(2,1,1)
plot(fx,fz);
hold on
comet(fx,fz);

subplot(2,1,2)
plot(px,py);
hold on
comet(px,py);

