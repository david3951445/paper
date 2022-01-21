 clc;clear;
%%%  bM(q)du+bH(q)=bB(q)tau %%% 
load dyn_para.mat
load J.mat
load dJdt.mat
syms v0 dphi0dt
v = [v0;dphi0dt];
bM = transpose(J)*M*J;
bM = simplify(bM);
bH = transpose(J)*H + transpose(J)*M*dJdt*v;
bH = simplify(bH);
bB = transpose(J)*B;
bB = simplify(bB);

save dyn_para_b.mat