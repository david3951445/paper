function gait = leg_gait_test(v)
%%% x(t) = c*v*sin(pi/T*(t-T/2)); %%%
%%% z(t) = hf/2*(1+sin(2*pi/T*(t-T/4))); %%%
%%% th = pi/T*(t-T/2) %%%
%%% one full step: t = 0 ~ 2T --> th = -pi/2 ~ 3*pi/2 %%%
h = 0.001;
% th = linspace(-pi/2,3*pi/2);
th = -pi/2:h:3*pi/2;
c = 0.2;
hf = 0.1;
%%% x-direction %%%
gait_px = c*v*sin(th);
gait_vx = c*v*cos(th);
gait_ax = -c*v*sin(th);
%%% z-direction %%%
for i = 1:length(th)
    if th(i) <= pi/2
        gait_pz(i) = hf/2*(1+sin(2*th(i)+pi/2));
    else
        gait_pz(i) = 0;
    end
end

for i = 1:length(th)
    if th(i) <= pi/2
        gait_vz(i) = hf*cos(2*th(i)+pi/2);
    else
        gait_vz(i) = 0;
    end
end

for i = 1:length(th)
    if th(i) <= pi/2
        gait_az(i) = -2*hf*sin(2*th(i)+pi/2);
    else
        gait_az(i) = 0;
    end
end


gait = [gait_px;gait_vx;gait_ax;gait_pz;gait_vz;gait_az];
end 