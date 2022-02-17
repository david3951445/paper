function [Weiner1 Weiner2 Weiner3 Weiner4 Weiner5]=mod_Weiner(L,dW1,dW2,dW3,dW4,dW5)
global Kx Ky Kz Kphi Ktheta Kpsi Jphi Jtheta Jpsi m g Ar mar_stable num
global pv1 pv2 pv3 Local
load multi_UAV.mat;
load stocastic_item.mat;
load UAV1.mat;
load UAV2.mat;
load UAV3.mat;
load UAV4.mat;
R=2;
for i=1:L
    Weiner1(i) = sum(dW1(1:i));
    Weiner2(i) = sum(dW2(1:i));
    Weiner3(i) = sum(dW3(1:i));
    Weiner4(i) = sum(dW4(1:i));
    Weiner5(i) = sum(dW5(1:i));
end
