%%% 計算尤拉角轉換的矩陣 %%%
function T = ET(d)
% unit: degree
th = d(2);
psi = d(3);
T = [cosd(phi)*cosd(th) cosd(phi)*sind(th)*sind(psi)-sind(phi)*cosd(psi) cosd(phi)*sind(th)*cosd(psi)+sind(phi)*sind(psi) 0;
     sind(phi)*cosd(th) sind(phi)*sind(th)*sind(psi)+cosd(phi)*cosd(psi) sind(phi)*sind(th)*cosd(psi)-cosd(phi)*sind(psi) 0;
             -sind(th)                            cosd(th)*sind(psi)                            cosd(th)*cosd(psi) 0;
                    0                                           0                                           0 1];
end