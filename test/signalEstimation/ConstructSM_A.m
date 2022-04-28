function Aa = ConstructSM_A(WINDOW, dt)
%CONSTRUCT SMooth Model's A matrix

%% Method 1
Aa = zeros(WINDOW);
% method 1-1
%     point = 0 : -1 : -WINDOW+1;
%     Aa(1, 1:WINDOW) = FindFDC(point, 1)'/dt;
% method 1-2
%     point = zeros(WINDOW, 1); point(1:2) = [1 -1];
%     Aa(1, 1:WINDOW) = point/dt;
% method 1-3
coeff2 = 0.1.^(0:WINDOW-1);
coeff2 = coeff2/sum(coeff2);
Aa(1, 1:WINDOW) = zeros(1, WINDOW); Aa(1,1) = -1;
Aa(1, 1:WINDOW) = (Aa(1, 1:WINDOW) + coeff2)/dt;

for i = 2 : WINDOW
    point = [1 0];
    Aa(i, i-1:i) = FindFDC(point, 1)'/dt; % obtain coefficient
end
    
%% Method 2
% for i = 1 : WINDOW
%     point = i-1 : -1 : -WINDOW+i;
%     Aa(i, :) = FindFDC(point, 1)'/dt;
% end

%% Method 3

end

