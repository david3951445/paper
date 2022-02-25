function gait = leg_gait(v,th,T)
%%% x(t) = c*v*sin(pi/T*(t-T/2)); %%%
%%% z(t) = hf/2*(1+sin(2*pi/T*(t-T/4))); %%%
%%% th = pi/T*(t-T/2) %%%
%%% one full step: t = 0 ~ 2T --> th = -pi/2 ~ 3*pi/2 %%%
% th = linspace(-pi/2,3*pi/2);
c = 0.2;
hf = 0.01;
n = floor((th+pi/2)/(2*pi));
th = th - n*2*pi;
%%% x-direction %%%
gait_px = c*v*sin(th);
gait_vx = c*v*cos(th)*pi/T;
gait_ax = -c*v*sin(th)*(pi/T)^2;
%%% z-direction %%%
if th <= pi/2
    gait_pz = hf/2*(1+sin(2*th+pi/2));
else
    gait_pz = 0;
end

if th <= pi/2
%     gait_vz = hf*cos(2*th+pi/2)*pi/T;
    gait_vz = hf*cos(2*th+pi/2);
else
    gait_vz = 0;
end

if th <= pi/2
%     gait_az = -2*hf*sin(2*th+pi/2)*(pi/T)^2;
    gait_az = -2*hf*sin(2*th+pi/2);
else
    gait_az = 0;
end



gait = [gait_px;gait_vx;gait_ax;gait_pz;gait_vz;gait_az];
end 