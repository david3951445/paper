function gait = leg_gait_s(hf,c,len,v,index,dt)

%%% generate gait with changed velocity %%% 
%%% by cubic spline %%%

%%% p %%%
S = c*v; % one step gait
th = [pi pi/2 0 -pi/2 -pi] ;
x = [-S 0 S 0 -S];
thq = linspace(pi,-pi,len);
xq = spline(th,x,thq);

for i = 1:length(thq)
    if thq(i) >= 0
        zq(i) = hf/2*(1+sin(2*thq(i)-pi/2));
    else
        zq(i) = 0;
    end
end

%%% v %%%
dth = thq(1)-thq(2); %%% vx = dx/dth * dth/dt
vx = diff(xq)/dt;
vx = [vx vx(1)]; %% since it is period function 
vz = diff(zq)/dt;
vz = [vz vz(1)];
%%% a %%%
ax = diff(vx)/dt;
ax = [ax ax(1)];
az = diff(vz)/dt;
az = [az az(1)];


gait=[xq(index);vx(index);ax(index);zq(index);vz(index);az(index)];

end
