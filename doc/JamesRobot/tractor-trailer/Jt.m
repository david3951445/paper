function J = Jt(phi,d)
for i = 1:length(phi)
    eval(['phi' num2str(i)-1 ' = phi(i);']);
end
J = [cos(phi1)*cos(phi0-phi1) 0;
     sin(phi1)*cos(phi0-phi1) 0;
           1/d*sin(phi0-phi1) 0;
                            0 1;];
end