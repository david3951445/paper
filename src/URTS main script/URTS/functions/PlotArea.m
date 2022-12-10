function PlotArea(area, name, faceColor)
    % Show the area in URTS
    
    vertex = combvec(area(1, :), area(2, :), area(3, :));
    k = 1;
    for i = 1 : size(area, 1)
        for j = 1 : size(area, 2)
            face(k, :) = find(vertex(i, :) == area(i, j));
            face(k, 3:4) = flip(face(k, 3:4));
            k = k + 1;
        end
    end
    patch('Faces',face,'Vertices',vertex','FaceColor',faceColor, FaceAlpha=.1, EdgeColor=[.8 .8 .8], DisplayName=name);
end