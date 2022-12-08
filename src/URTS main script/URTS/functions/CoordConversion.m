function newCoords = CoordConversion(coords)
    rotZMinus90 = [0 1; -1 0];
    for j = 1 : size(coords, 2)
        coords(:, j) = rotZMinus90*coords(:, j); % Rotete Z-axis -90
        coords(2, j) = coords(2, j) + 6; % Translate Y axis +6
    end
    newCoords = coords;
end

