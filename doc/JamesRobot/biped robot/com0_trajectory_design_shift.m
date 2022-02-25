function com0_trajectory_design_shift(team,robot)
eval(['F1 = load("com0_trajectory_reference_model_team1_follower1.mat");']);

h = F1.h;
hh = F1.hh;
tr = F1.tr;
t = F1.t;
nv = F1.nv;
t_end = F1.t_end;
T1 = F1.t1;
T2 = F1.t16;

Period = 3;
ShiftTime = 6*Period*(team-1) + Period*(robot-1);

shift = F1.r(:,(T1+ShiftTime)/hh) - F1.r(:,T1/hh);
tmp1 = F1.r(:,T1/hh+1:(T1+ShiftTime)/hh) - F1.r(:,T1/hh);
tmp2 = F1.r(:,(T1+ShiftTime)/hh+1:T2/hh) - F1.r(:,(T1+ShiftTime)/hh);
tmp3 = F1.r(:,T2/hh+1:end) - F1.r(:,T2/hh);

r(:,1:T1/hh) = F1.r(:,1:T1/hh) + shift;
r(:,T1/hh+1:(T2-ShiftTime)/hh) = tmp2 + r(:,T1/hh);
r(:,(T2-ShiftTime)/hh+1:T2/hh) = tmp1 + r(:,(T2-ShiftTime)/hh);
r(:,T2/hh+1:t_end/hh+1) = tmp3 + r(:,T2/hh);

beta = 10;
Ar = -beta*eye(nv);
Br = beta*eye(nv);

xr(:,1) = r(:,1);

for i = 1:length(tr)-1
    k1 = com0_RungeKutta(Ar,Br,xr(:,i),r(:,2*i-1));
    k2 = com0_RungeKutta(Ar,Br,xr(:,i)+h*k1/2,r(:,2*i));
    k3 = com0_RungeKutta(Ar,Br,xr(:,i)+h*k2/2,r(:,2*i));
    k4 = com0_RungeKutta(Ar,Br,xr(:,i)+h*k3,r(:,2*i+1));
    xr(:,i+1) = xr(:,i) + 1/6*h*(k1+2*k2+2*k3+k4);
    disp(i);
end

eval(['save("com0_trajectory_reference_model_team' num2str(team) '_follower' num2str(robot) '.mat","xr","r","t","tr","hh","h","nv");']);
end