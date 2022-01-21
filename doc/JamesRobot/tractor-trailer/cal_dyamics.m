clc;clear;
%%%  M(q)du+H(q)=B(q)tau+A(q)'lambda %%% 
syms phi0 phi1 dphi0dt dphi1dt m0 m1 d a1 a0 I0 I1 r b

m = m0 + m1;
A = a1*m1 + d*m0;
Iphi1 = m1*a1^2 + m0*d^2 + I1;
Iphi0 = m1*a0^2 + I0;

M = [               m               0       -a0*m0*sin(phi0)           -A*sin(phi1);
                    0               m        a0*m0*cos(phi0)            A*cos(phi1);
     -a0*m0*sin(phi0) a0*m0*cos(phi0)                  Iphi1 a0*d*m0*cos(phi0-phi1);
         -A*sin(phi1)    -A*cos(phi1) a0*d*m0*cos(phi0-phi1)                 Iphi1];
B = 1/r*[      cos(phi0)       cos(phi0); 
               sin(phi0)       sin(phi0);
                       b              -b;
        d*sin(phi0-phi1) d*sin(phi0-phi1)];
H11 = -a0*m0*cos(phi0)*dphi0dt^2 - A*cos(phi1)*dphi1dt^2;
H21 = -a0*m0*sin(phi0)*dphi0dt^2 - A*sin(phi1)*dphi1dt^2;
H31 = a0*d*m0*dphi1dt^2*sin(phi0-phi1);
H41 = -a0*d*m0*dphi0dt^2*sin(phi0-phi1);
H = [H11;H21;H31;H41];
save dyn_para.mat