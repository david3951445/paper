function Je = leg_Je(dthdt,th,L4,L5)
for i = 1:length(dthdt)
eval(['th' num2str(i) '=th(i);']);
eval(['dth' num2str(i) 'dt=dthdt(i);']);
end
Je = blkdiag(leg_Jb_inv(th,L4,L5),ones(4));
end