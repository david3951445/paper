function [p,v,a] = com0_quadratic(B,dB,dC,tacc,h,T)
 a4 = - (tacc*dC/T - dB);
 a3 = 2*(tacc*dC/T - dB);
 a2 = 0;
 a1 = 2*dB;
 a0 = B-dB;
 p =  a4*h^4    + a3*h^3   + a2*h^2 + a1*h     +a0  ;
 v = (4*a4*h^3  + 3*a3*h^2 + 2*a2*h + a1)/(2*tacc)  ;
 a = (12*a4*h^2 + 6*a3*h   + 2*a2       )/(2*tacc)^2;
end