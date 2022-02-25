function dJedt = leg_dJedt(dthdt,th,L4,L5)
for i = 1:length(dthdt)
eval(['th' num2str(i) '=th(i);']);
eval(['dth' num2str(i) 'dt=dthdt(i);']);
end
dJedt = blkdiag(leg_dinvJbdt(dthdt,th,L4,L5),zeros(4));
end