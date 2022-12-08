function PlotArea(area1)
    % Show the area1 in URTS
    
    % area1 = [0 6; 0 6; 0 4];
    vertex = combvec(area1(1, :), area1(2, :), area1(3, :));
    k = 1;
    for i = 1 : size(area1, 1)
        for j = 1 : size(area1, 2)
            face(k, :) = find(vertex(i, :) == area1(i, j));
            face(k, 3:4) = flip(face(k, 3:4));
            k = k + 1;
        end
    end
    patch('Faces',face,'Vertices',vertex','FaceColor','r', FaceAlpha=.1, EdgeColor=[.8 .8 .8], DisplayName='$area_1$');
end