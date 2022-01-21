clc;clear;

h = 1e-4;
vxr = 0.1;
vyr = 0.06;
vxr1 = vxr + 2*h;
vxr2 = vxr + h;
vxr3 = vxr - h;
vxr4 = vxr - 2*h;


v1 = vxr1^2+vyr^2;
v2 = vxr2^2+vyr^2;
v3 = vxr3^2+vyr^2;
v4 = vxr4^2+vyr^2;

output1 = f(v1,2000);
output2 = f(v2,2000);
output3 = f(v3,2000);
output4 = f(v4,2000);
%%
ddq1 = output1(29:36);
ddq2 = output2(29:36);
ddq3 = output3(29:36); 
ddq4 = output4(29:36);

ddq111 = -ddq1(5)-ddq1(7);
ddq112 = -ddq2(5)-ddq2(7);
ddq113 = -ddq3(5)-ddq3(7);
ddq114 = -ddq4(5)-ddq4(7);


d = (-ddq111 + 8*ddq112 - 8*ddq113 + ddq114) / (12*h);