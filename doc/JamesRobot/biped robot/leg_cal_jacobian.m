clc;clear;
load Jbl.mat
load Jbr.mat

%%% Vleg,r = Jbr*[th2;th4;th6;th8] Vleg,l = Jbl*[th1;th3;th5;th7] %%%
Jb = [      0,   J11r,      0,   J12r,      0,   J13r,      0,   J14r;
            0,   J21r,      0,   J22r,      0,   J23r,      0,   J24r;
            0,   J31r,      0,   J32r,      0,   J33r,      0,   J34r;
            0,wb0r(3),      0,wb1r(3),      0,wb2r(3),      0,wb3r(3);
         J11l,      0,   J12l,      0,   J13l,      0,   J14l,      0;
         J21l,      0,   J22l,      0,   J23l,      0,   J24l,      0;
         J31l,      0,   J32l,      0,   J33l,      0,   J34l,      0;
      wb0l(3),      0,wb1l(3),      0,wb2l(3),      0,wb3l(3),      0];
Jb_inv = inv(Jb);
save Jb.mat