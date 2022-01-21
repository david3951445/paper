function pinvJ = pinvJt(phi,d)
for i = 1:length(phi)
    eval(['phi' num2str(i)-1 ' = phi(i);']);
end
pinvJ = [...
 (d^2*(cos(phi0 - 2*phi1) + cos(phi0)))/(d^2*cos(2*phi0 - 2*phi1) - cos(2*phi0 - 2*phi1) + d^2 + 1), -(d^2*(sin(phi0 - 2*phi1) - sin(phi0)))/(d^2*cos(2*phi0 - 2*phi1) - cos(2*phi0 - 2*phi1) + d^2 + 1), (2*d*sin(phi0 - phi1))/(d^2*cos(2*phi0 - 2*phi1) - cos(2*phi0 - 2*phi1) + d^2 + 1), 0;
                                                                                                  0,                                                                                                   0,                                                                                  0, 1];
end