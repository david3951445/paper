function dJdt = dJtdt(dphidt,phi,d)
for i = 1:length(phi)
    eval(['phi' num2str(i)-1 ' = phi(i);']);
    eval(['dphi' num2str(i)-1 'dt = dphidt(i);']);
end
dJdt = [- sin(phi1)*cos(phi0 - phi1)*dphi1dt - cos(phi1)*sin(phi0 - phi1)*(dphi0dt - dphi1dt), 0;
          cos(phi1)*cos(phi0 - phi1)*dphi1dt - sin(phi1)*sin(phi0 - phi1)*(dphi0dt - dphi1dt), 0;
                                                     (cos(phi0 - phi1)*(dphi0dt - dphi1dt))/d, 0;
                                                                                            0, 0];
end